// license:BSD-3-Clause
// copyright-holders:R. Belmont, tim lindner, Laurie Laptop
/***************************************************************************

    drivers/enmirage.c

    Ensoniq Mirage Sampler
    Preliminary driver by R. Belmont
    Fleshed out by tim lindner

    Models:
        DSK-8: Pratt-Reed keyboard (early 1984)
        DSK-8: Fatar keyboard (late 1984)
        DMS-8: Rack mount (1985)
        DSK-1: Unweighted keyboard, stereo output (1986)

    M6809 Map for Mirage:
        0000-7fff: 32k window on 128k of sample RAM
        8000-bfff: main RAM
        c000-dfff: optional expansion RAM
        e100-e101: 6850 UART (for MIDI)
        e200-e2ff: 6522 VIA
        e400-e407: write to both filters
        e408-e40f: filter resonance
        e410-e417: filter cut-off frequency
        e418-e41f: DAC pre-set
        (One DAC serves all the filter control voltages.  Each write loads the data
        into the DAC and latches the low address bits, which keep selecting the
        destination until the next write.  Address bit 3 inhibits the cut-off
        selector and bit 4 the resonance selector, so with neither set both are
        written, and with both set only the DAC is loaded.)
        e800-e803: WD1770 FDC
        ec00-ecef: ES5503 "DOC" sound chip
        f000-ffff: boot ROM

    M6809 Interrupts:
        NMI: IRQ from WD1772
        IRQ: wired-ORed: DRQ from WD1772, IRQ from ES5503, IRQ from VIA6522, IRQ from cartridge
        FIRQ: IRQ from 6850 UART

    LED / switch matrix:

            A           B           C             D         E         F         G        DP
    ROW 0:  LOAD UPPER  LOAD LOWER  SAMPLE UPPER  PLAY SEQ  LOAD SEQ  SAVE SEQ  REC SEQ  SAMPLE LOWER
    ROW 1:  3           6           9             5         8         0         2        Enter
    ROW 2:  1           4           7             up arrow  PARAM     dn arrow  VALUE    CANCEL
    L. AN:  SEG A       SEG B       SEG C         SEG D     SEG E     SEG F     SEG G    SEG DP (decimal point)
    R. AN:  SEG A       SEG B       SEG C         SEG D     SEG E     SEG F     SEG G    SEG DP

    Column number in VIA port A bits 0-2 is converted to discrete lines by a 74LS145.
    Port A bit 3 is right anode, bit 4 is left anode
    ROW 0 is read on VIA port A bit 5, ROW 1 in port A bit 6, and ROW 2 in port A bit 7.

    Keyboard models talk to the R6500/11 through the VIA shifter: CA2 is handshake, CB1 is shift clock,
    CB2 is shift data.
    This is unconnected on the rackmount version.  A stand-in for the R6500/11 is
    provided that plays a MIDI file (-kbdin) into the shifter.

    Unimplemented:
        * External sync signal
        * Foot pedal
        * Piano keyboard controller (only the stand-in described above)
        * Expansion connector
        * Stereo output

    Modelled approximately:
        * The eight CEM3328 four-pole low-pass filters (see "CEM3328 filters" below).
        * The DOC's ADC feedback: mux inputs 0 and 1 (the compressed and the line level
          mixed input) read the summed output of those filters, which is what the boot
          ROM's filter auto-tune measures (see "ADC feedback" below).  The compander in
          front of input 0 is not modelled, so the two read alike.

***************************************************************************/


#include "emu.h"
#include "bus/midi/midi.h"
#include "cpu/m6809/m6809.h"
#include "formats/esq8_dsk.h"
#include "formats/hxchfe_dsk.h"
#include "imagedev/cassette.h"
#include "imagedev/floppy.h"
#include "imagedev/midiin.h"
#include "machine/6522via.h"
#include "machine/6850acia.h"
#include "machine/clock.h"
#include "machine/input_merger.h"
#include "machine/wd_fdc.h"
#include "sound/es5503.h"
#include "speaker.h"
#include "video/pwm.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "enmirage.lh"

#define LOG_ADC_READ        (1U << 1)
#define LOG_FILTER_WRITE    (1U << 2)
#define LOG_KBD             (1U << 3)
#define VERBOSE (0)
//#define VERBOSE (LOG_ADC_READ)
//#define VERBOSE (LOG_ADC_READ|LOG_FILTER_WRITE)
//#define VERBOSE (LOG_KBD)

#include "logmacro.h"

#define LOGADCREAD(...)     LOGMASKED(LOG_ADC_READ, __VA_ARGS__)
#define LOGFILTERWRITE(...) LOGMASKED(LOG_FILTER_WRITE, __VA_ARGS__)
#define LOGKBD(...)         LOGMASKED(LOG_KBD, __VA_ARGS__)


namespace {

#define PITCH_TAG "pitch"
#define MOD_TAG "mod"

/***************************************************************************
    Keyboard controller stand-in

    The DSK-8's 61-key keyboard is scanned by an R6500/11 microcontroller
    that reports to the CPU through the VIA's shift register: the
    controller's SCLK output is the VIA's CB1 input, its SDATA output is
    CB2, and the VIA's CA2 output is the controller's /SACK handshake input.
    The rackmount DMS-8 has no keyboard, so nothing in this driver could
    originate a note.

    This device does not emulate the R6500/11, whose firmware is not
    available.  It reproduces the controller's output protocol instead, which
    the OS fixes completely:

    * Messages are MIDI-like: 0x90 key velocity for note on, 0x80 key
      velocity for note off, 0xb8 for sustain pedal down and 0xb9 for pedal
      up.  The pedal messages carry no data bytes.
    * Key numbers count from 0 at the bottom of the 61 keys, and the OS adds
      0x24, so key 0 is MIDI note 36 and key 60 is MIDI note 96.
    * The velocity byte is the controller's raw timing count.  The OS maps it
      through a curve in the boot ROM (one table for attack at 0xfbf8, one
      for release at 0xfbfe), adds the sensitivity setting less 0x1e and
      clamps the result to 0x01-0x7f.  The device finds the raw byte that
      gives a requested MIDI velocity by inverting the curve in the loaded
      ROM region at start-up, so no ROM data is needed in this file.
    * One byte is delivered per VIA shift register interrupt (ACR = 0xcc:
      shift in under the external clock on CB1).  After each byte the OS
      pulses CA2 low then high to release the next one, and withholds the
      pulse when its event pool runs low, which is the only flow control.
      A byte is therefore shifted out only after a falling edge on /SACK
      has been seen since the previous byte.

    These parts of the protocol are assumed rather than known, since the
    controller's firmware cannot be inspected: the bit order and polarity
    (most significant bit first, positive logic, which follows from the 6522
    shifting MSB first and the OS using the byte unmodified), the bit clock
    (10us per bit, which the OS is insensitive to), the controller's
    reaction time after /SACK (50us, which must exceed the roughly 15 CPU
    cycles between the OS handing over and discarding its first read of the
    shift register), and that every event begins with a status byte rather
    than using MIDI-style running status (the OS accepts either).

    Input is a MIDI file given with -kbdin.  A midiin image device parses the
    file and streams it as MIDI serial data, and this device decodes the
    bytes and translates note on/off and controller 64 into keyboard
    controller events.  The ACIA's own -midiin option is not affected, so a
    file can drive either the MIDI IN port or the keyboard.

    The velocity curves come from the boot ROM, so -kbdin refuses its image
    if the ROM does not have curve-shaped tables where this device looks.

***************************************************************************/

// Where the boot ROM keeps its velocity curves.  Each is a table indexed by
// the controller's raw byte, with the release table starting six entries
// after the attack table.  Only the monotonic prefix is the curve - past it
// the table runs into code - so a curve ends at the first decrease or at a
// value above 0x7f.  The check is that something curve-shaped is there
// (long enough, and wide enough to cover most of the MIDI velocity range),
// not which ROM it is: another revision with these tables in these places
// is accepted, one without them is not.
struct velocity_curve
{
	unsigned offset;
	const char *name;
};

const velocity_curve VELOCITY_CURVES[2] = {
	{ 0xbf8, "attack"  },
	{ 0xbfe, "release" },
};

constexpr unsigned MIN_CURVE_LENGTH = 64;
constexpr unsigned MIN_CURVE_RANGE  = 64;

// Length of the curve at an offset in the boot ROM, or 0 if there is none
unsigned velocity_curve_length(memory_region *rom, unsigned offset)
{
	if (!rom || rom->bytes() <= offset)
		return 0;

	const uint8_t *curve = rom->base() + offset;
	unsigned len = 1;
	while (len < 0x100 && (offset + len) < rom->bytes() && curve[len] >= curve[len - 1] && curve[len] <= 0x7f)
		len++;
	if (curve[0] > 0x7f || len < MIN_CURVE_LENGTH || (curve[len - 1] - curve[0]) < MIN_CURVE_RANGE)
		return 0;
	return len;
}

// The MIDI file image behind -kbdin.  This is a midiin device under another
// type so that the ACIA's port keeps the option name -midiin instead of
// both being numbered.
class mirage_kbdin_device : public midiin_device
{
public:
	mirage_kbdin_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	virtual std::pair<std::error_condition, std::string> call_load() override;

	virtual const char *image_type_name() const noexcept override { return "kbdin"; }
	virtual const char *image_brief_type_name() const noexcept override { return "kbd"; }
};

DEFINE_DEVICE_TYPE_PRIVATE(MIRAGE_KBDIN, mirage_kbdin_device, mirage_kbdin_device, "mirage_kbdin", "Mirage keyboard MIDI file input")

class mirage_keyboard_device : public device_t, public device_serial_interface
{
public:
	mirage_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	auto sclk_handler() { return m_sclk_cb.bind(); }    // -> VIA CB1
	auto sdata_handler() { return m_sdata_cb.bind(); }  // -> VIA CB2
	void sack_w(int state);                              // <- VIA CA2

	// Protocol timing in microseconds (assumed, see above)
	static constexpr int BIT_HALF_PERIOD_US = 5;
	static constexpr int SACK_LATENCY_US    = 50;

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_serial_interface implementation (MIDI bytes arrive at 31250 8-N-1)
	virtual void rcv_complete() override;

private:
	enum : uint8_t { IDLE, LATENCY, CLK_LOW, CLK_HIGH };

	TIMER_CALLBACK_MEMBER(shift_tick);
	void midi_byte(uint8_t data);
	void midi_message();
	void enqueue(uint8_t data);
	void arm_if_ready();
	void build_velocity_inverse();
	uint8_t velocity_to_raw(uint8_t vel, bool release) const;

	required_device<midiin_device> m_midi;
	devcb_write_line m_sclk_cb;
	devcb_write_line m_sdata_cb;
	emu_timer *m_timer;

	// bytes waiting for the link, oldest first
	uint8_t m_ring[256];
	uint8_t m_rd, m_wr;

	// the shifter
	uint8_t m_phase;
	uint8_t m_byte;
	uint8_t m_bit;
	bool m_armed;      // a /SACK pulse has been seen since the last byte
	int m_sack;        // the CA2 line as last driven

	// the MIDI parser
	uint8_t m_status;
	uint8_t m_data[2];
	uint8_t m_ndata;
	bool m_pedal;

	// raw timing byte for each MIDI velocity, from the ROM curve
	uint8_t m_raw_attack[128];
	uint8_t m_raw_release[128];
};

DEFINE_DEVICE_TYPE_PRIVATE(MIRAGE_KEYBOARD, mirage_keyboard_device, mirage_keyboard_device, "mirage_kbd", "Mirage keyboard controller (R6500/11 protocol)")

mirage_kbdin_device::mirage_kbdin_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: midiin_device(mconfig, MIRAGE_KBDIN, tag, owner, clock)
{
}

std::pair<std::error_condition, std::string> mirage_kbdin_device::call_load()
{
	// the keyboard device needs the boot ROM's velocity curves, so do not accept input without them
	memory_region *rom = machine().root_device().memregion("osrom");
	for (const auto &c : VELOCITY_CURVES)
	{
		if (!velocity_curve_length(rom, c.offset))
		{
			return std::make_pair(
					image_error::INVALIDIMAGE,
					util::string_format("The boot ROM has no %s velocity curve at $%04x, which the keyboard input needs", c.name, 0xf000 + c.offset));
		}
	}

	return midiin_device::call_load();
}

mirage_keyboard_device::mirage_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MIRAGE_KEYBOARD, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, m_midi(*this, "midi")
	, m_sclk_cb(*this)
	, m_sdata_cb(*this)
	, m_timer(nullptr)
	, m_rd(0), m_wr(0)
	, m_phase(IDLE), m_byte(0), m_bit(0), m_armed(false), m_sack(1)
	, m_status(0), m_ndata(0), m_pedal(false)
{
}

void mirage_keyboard_device::device_add_mconfig(machine_config &config)
{
	MIRAGE_KBDIN(config, m_midi, 0);
	m_midi->input_callback().set(FUNC(mirage_keyboard_device::rx_w));
}

void mirage_keyboard_device::device_start()
{
	m_timer = timer_alloc(FUNC(mirage_keyboard_device::shift_tick), this);
	build_velocity_inverse();

	save_item(NAME(m_ring));
	save_item(NAME(m_rd));
	save_item(NAME(m_wr));
	save_item(NAME(m_phase));
	save_item(NAME(m_byte));
	save_item(NAME(m_bit));
	save_item(NAME(m_armed));
	save_item(NAME(m_sack));
	save_item(NAME(m_status));
	save_item(NAME(m_data));
	save_item(NAME(m_ndata));
	save_item(NAME(m_pedal));
}

void mirage_keyboard_device::device_reset()
{
	// we only receive: MIDI is 31250 8-N-1
	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rcv_rate(31250);
	set_tra_rate(0);
	receive_register_reset();

	m_rd = m_wr = 0;
	m_phase = IDLE;
	m_armed = false;
	m_status = 0;
	m_ndata = 0;
	m_pedal = false;
	m_timer->adjust(attotime::never);

	// idle: clock high, data high.  The 6522 shifts in on the rising CB1 edge
	// and counts edges from the SR read, so the clock must rest high.
	m_sclk_cb(1);
	m_sdata_cb(1);
}

// Invert the boot ROM's velocity curves.  For each MIDI velocity the nearest
// curve value wins, which is the best any keyboard can do: with the default
// sensitivity the OS passes on exactly curve[raw].  If a curve is missing the
// table is left empty; that cannot matter, because mirage_kbdin_device
// refused its image and nothing is sent to this device.
void mirage_keyboard_device::build_velocity_inverse()
{
	std::fill(std::begin(m_raw_attack), std::end(m_raw_attack), 0);
	std::fill(std::begin(m_raw_release), std::end(m_raw_release), 0);

	memory_region *rom = machine().root_device().memregion("osrom");
	uint8_t *const outputs[2] = { m_raw_attack, m_raw_release };
	for (int i = 0; i < 2; i++)
	{
		const velocity_curve &c = VELOCITY_CURVES[i];
		const unsigned len = velocity_curve_length(rom, c.offset);
		if (!len)
		{
			LOGKBD("%s: no %s curve in the boot ROM\n", tag(), c.name);
			continue;
		}

		const uint8_t *curve = rom->base() + c.offset;
		LOGKBD("%s: %s curve at ROM $%04x, %u monotonic entries, %02x..%02x\n",
				tag(), c.name, 0xf000 + c.offset, len, curve[0], curve[len - 1]);
		for (unsigned vel = 0; vel < 128; vel++)
		{
			unsigned want = vel ? vel : 1;
			unsigned best = 0, bestdist = 0x100;
			for (unsigned r = 0; r < len; r++)
			{
				unsigned dist = (curve[r] > want) ? (curve[r] - want) : (want - curve[r]);
				if (dist < bestdist) { bestdist = dist; best = r; }
			}
			outputs[i][vel] = uint8_t(best);
		}
	}
}

uint8_t mirage_keyboard_device::velocity_to_raw(uint8_t vel, bool release) const
{
	return release ? m_raw_release[vel & 0x7f] : m_raw_attack[vel & 0x7f];
}

// --- the serial link to the VIA ---

void mirage_keyboard_device::sack_w(int state)
{
	if (state == m_sack)
		return;
	m_sack = state;
	if (!state)
	{
		// the OS pulses CA2 low then high after each byte.  The low edge is
		// the acknowledge; the next byte may go.
		LOGKBD("%s: /SACK pulse\n", tag());
		m_armed = true;
		arm_if_ready();
	}
}

void mirage_keyboard_device::enqueue(uint8_t data)
{
	if (uint8_t(m_wr + 1) == m_rd)
	{
		logerror("%s: keyboard queue full, byte $%02x dropped\n", tag(), data);
		return;
	}
	m_ring[m_wr++] = data;
	arm_if_ready();
}

void mirage_keyboard_device::arm_if_ready()
{
	if (m_phase != IDLE || !m_armed || m_rd == m_wr)
		return;
	m_phase = LATENCY;
	m_timer->adjust(attotime::from_usec(SACK_LATENCY_US));
}

TIMER_CALLBACK_MEMBER(mirage_keyboard_device::shift_tick)
{
	switch (m_phase)
	{
	case LATENCY:
		// start the byte: MSB first, data set up before the clock falls
		m_byte = m_ring[m_rd++];
		m_armed = false;
		m_bit = 8;
		LOGKBD("%s: shifting $%02x\n", tag(), m_byte);
		[[fallthrough]];

	case CLK_HIGH:
		if (m_bit == 0)
		{
			// byte complete; wait for the OS to acknowledge it
			m_phase = IDLE;
			arm_if_ready();
			return;
		}
		m_bit--;
		m_sdata_cb(BIT(m_byte, m_bit));
		m_sclk_cb(0);
		m_phase = CLK_LOW;
		m_timer->adjust(attotime::from_usec(BIT_HALF_PERIOD_US));
		return;

	case CLK_LOW:
		// the rising edge is where the 6522 samples SDATA
		m_sclk_cb(1);
		m_phase = CLK_HIGH;
		m_timer->adjust(attotime::from_usec(BIT_HALF_PERIOD_US));
		return;

	default:
		m_phase = IDLE;
		return;
	}
}

// --- MIDI in -> keyboard events ---

void mirage_keyboard_device::rcv_complete()
{
	receive_register_extract();
	midi_byte(get_received_char());
}

void mirage_keyboard_device::midi_byte(uint8_t data)
{
	if (data >= 0xf8)
		return;                          // real-time: nothing a keyboard does
	if (data >= 0xf0)
	{
		m_status = 0;                    // system common cancels running status
		m_ndata = 0;
		return;
	}
	if (data & 0x80)
	{
		m_status = data;
		m_ndata = 0;
		return;
	}
	if (!m_status)
		return;                          // data with no status: ignore
	m_data[m_ndata++] = data;
	const unsigned need = ((m_status & 0xf0) == 0xc0 || (m_status & 0xf0) == 0xd0) ? 1 : 2;
	if (m_ndata >= need)
	{
		midi_message();
		m_ndata = 0;                     // running status stays in force
	}
}

void mirage_keyboard_device::midi_message()
{
	const uint8_t kind = m_status & 0xf0;
	if (kind == 0x90 || kind == 0x80)
	{
		const bool off = (kind == 0x80) || (m_data[1] == 0);
		const int key = int(m_data[0]) - 0x24;   // the OS adds 0x24 back
		if (key < 0 || key > 60)
		{
			LOGKBD("%s: MIDI note %d is off the 61-key keyboard, ignored\n", tag(), m_data[0]);
			return;
		}
		const uint8_t raw = velocity_to_raw(m_data[1], off);
		LOGKBD("%s: note %s key %d vel %d -> raw $%02x\n", tag(), off ? "off" : "on", key, m_data[1], raw);
		enqueue(off ? 0x80 : 0x90);
		enqueue(uint8_t(key));
		enqueue(raw);
	}
	else if (kind == 0xb0 && m_data[0] == 64)
	{
		const bool down = m_data[1] >= 64;
		if (down != m_pedal)
		{
			m_pedal = down;
			LOGKBD("%s: pedal %s\n", tag(), down ? "down" : "up");
			enqueue(down ? 0xb8 : 0xb9);
		}
	}
	// everything else (other controllers, program change, bend) has no
	// keyboard controller equivalent; the wheels reach the OS through the ADC
}


/***************************************************************************
    CEM3328 filters

    The eight CEM3328 filters, one per voice, are the Mirage's entire analog
    voice.  There is no VCA on the board and none inside the chip, so
    amplitude is the DOC's own per-oscillator volume register, which
    es5503.cpp already applies.  Without this device the DOC reaches the
    speaker unfiltered.

    The routing this hangs off exists on both sides.  The DOC presents a
    channel address and a 1-of-8 analog switch feeds that channel's output to
    the corresponding filter.  MAME's es5503 assigns each oscillator to
    output channel (control >> 4) & (channels - 1) and this driver sets
    eight channels, so the per-voice input is simply the eight stream
    outputs.

    What the model carries:

    * Four poles at one frequency with resonance feedback, solved with zero
      delay in the loop so that the oscillation threshold sits at exactly a
      loop gain of 4 at every cutoff rather than drifting with sample rate.
    * The datasheet's limiter, in the feedback path where the chip's block
      diagram draws it, bounding self-oscillation instead of letting the
      loop diverge.
    * Sample-and-hold on both control voltages, with the settling of a
      single DAC strobe: one write moves the hold capacitor 80-95% of the
      way, so a cutoff step arrives over two or three control passes as it
      does on the instrument.
    * The 12dB passband drop at full resonance, which is what the Mirage
      gets because the chip's resonance compensation is not used.
    * The chip's output noise, at the datasheet figure.  Without something to
      amplify, a digital model of a marginally unstable loop sits at exactly
      zero forever and never oscillates.

    What it does not carry:

    * The passive low-pass at the filter input.  It is a fixed pole well
      above the audio band.
    * Any per-chip spread.  The datasheet's initial frequency and resonance
      spreads are what the boot ROM's calibration exists to trim.
    * The chip's drift and temperature coefficient.  (Its noise is modelled,
      see NOISE_RMS_VOLTS.)

    This is an approximation to save trips to the real instrument, not a
    measurement of its tone.

***************************************************************************/

// ---- the CV path: DAC byte to the chip's frequency and resonance inputs ----

// Full-scale voltage of the sample-and-holds, set by the DAC reference that
// is derived from the regulated analog supply rails.  The exact reference is
// not known, and it scales the whole cutoff range.  If measurements are
// consistently sharp or flat, look at F_ZERO_CAP_SCALE first, which is known
// to a component value, before this.
constexpr double CV_V_SPAN = 5.0;

// A resistor network between the sample-and-hold and the chip's frequency
// control input divides the voltage by about 28 and adds an offset of about
// -50mV.
constexpr double CV_DIVIDER  = 28.0;
constexpr double CV_OFFSET_V = -0.050;

// A DAC write leaves the route to a hold capacitor open for the five or six
// CPU cycles until the next write, which moves the capacitor 80-95% of the
// way to the new value.  Set to 1.0 for an idealised model with instant CVs.
constexpr double CV_SETTLE_K = 0.85;

// ---- the chip, from the CEM3328 datasheet ----

// Frequency control scale: 20mV per octave (typical) over a 14 octave range.
constexpr double F_VOLTS_PER_OCTAVE = 0.020;

// Initial frequency at zero control voltage: 350Hz (typical), specified with
// 0.03uF pole capacitors.  The Mirage fits 0.033uF for the three main poles
// and frequency goes as 1/C, so the board sits 0.030/0.033 below the
// specified figure, about 0.14 octave.  Kept separate from CV_V_SPAN because
// it is known to a component value and CV_V_SPAN is not.
constexpr double F_ZERO_DATASHEET_HZ = 350.0;
constexpr double F_ZERO_CAP_SCALE    = 0.030 / 0.033;
constexpr double F_ZERO_HZ           = F_ZERO_DATASHEET_HZ * F_ZERO_CAP_SCALE;

// This is the largest assumption in the model: F_ZERO_HZ is taken to be the
// corner frequency of each individual pole, not the -3dB point of the four
// together.  The datasheet only gives an initial frequency and specifies it in
// terms of the pole capacitors, which is what sets a transconductor and
// capacitor pole, so this is the physically motivated reading but it is a
// reading.  A cascade of four equal poles is -3dB at 0.435 of the pole
// corner, so the two readings differ by a fixed 1.20 octaves.  If the model
// turns out to be uniformly 1.2 octaves out, this is the reason and
// F_ZERO_HZ is where to fix it.

// Resonance control voltage for oscillation: 3.2V (typical, 2.7V to 3.7V).
// The resonance CV is the sample-and-hold output directly, so with a 5V span
// that is DAC byte 3.2/5 * 255 = 163, and a four pole loop reaches its
// oscillation gain at exactly 4, which fixes the mapping.
constexpr double Q_OSC_BYTE = 163.0;

// Passband gain change from zero to maximum resonance: -12dB, at a ratio of
// resonance to signal input of zero, which is where the Mirage sits.  The
// datasheet curve for this is plotted against that input ratio, which is a
// circuit design choice rather than a played control, so it gives the
// endpoint and nothing about the way there.  Linear in dB against the loop
// gain is this model's own choice; if it sounds wrong the shape has to come
// from measuring the instrument.
constexpr double PASSBAND_DROP_DB = -12.0;

// The limiter's bound.  The datasheet gives an oscillation output swing of
// 2.3V peak to peak (typical) against a nominal output swing of 6.5V peak to
// peak for 1% THD without resonance, so a self-oscillating chip settles about
// 9dB below its own clean full output.  Taking full scale at this device's
// input and output to be that 6.5V, the target is 2.3/6.5 of it.  Simulating
// this loop at maximum resonance and solving for the bound that gives that
// amplitude yields the value below.  It is linear in the bound, so it can be
// re-derived in one step if the mapping changes.
constexpr double Q_LIMIT = 0.10146;

// es5503.cpp scales each of its eight channel outputs by 32768*8, reserving
// headroom for all eight summing to full scale, so one voice at full DOC
// volume arrives at 1/8.  Undo that on the way in so that a full-scale voice
// means the chip's nominal 6.5V, which is what makes Q_LIMIT a datasheet
// quantity, and put it back after the summer so that the overall level is
// unchanged when the filters are wide open.
constexpr double DOC_CHANNEL_HEADROOM = 8.0;
constexpr double SUMMER_SCALE         = 1.0 / DOC_CHANNEL_HEADROOM;

// The consequence of that choice, stated once: 1.0 peak in this device is
// half of 6.5V peak to peak.
constexpr double VOLTS_PER_UNIT = 6.5 / 2.0;

// Output noise with the filter wide open: 200uV r.m.s. (a datasheet maximum;
// there is no typical figure).  This is not decoration: the boot ROM's filter
// auto-tune measures a self-oscillation, and a zero-delay resonance loop
// driven by exactly 0.0 stays at exactly 0.0 however far past its threshold
// the gain is pushed.  A real chip oscillates because it has noise to
// amplify.  The noise is injected at the input, inside the resonance loop, so
// it is shaped by the four poles and grows at the loop's own rate.
//
// It is about 144dB below full scale, three orders of magnitude under a
// 16-bit LSB, so it cannot reach a recording.  What it does is take about
// fourteen cycles of oscillation to seed the loop up to the limiter, which is
// 23ms at the frequency the ROM tunes to.  Set to 0.0 to remove it, at the
// cost of a calibration that can only time out.
constexpr double NOISE_RMS_VOLTS = 200e-6;
constexpr double NOISE_RMS_UNITS = NOISE_RMS_VOLTS / VOLTS_PER_UNIT;

// At byte 255 the CV law asks for about 27kHz, which is past Nyquist at any
// ordinary output rate and would send the bilinear prewarp through infinity.
// Clamp it: the filter is wide open either way.
constexpr double FC_MAX_FRACTION = 0.45;

// ---- ADC feedback ----
//
// The DOC's own 8-bit converter (register 0xe2) reads the machine's analog
// output back.  Mux input 0 is the compressed and mixed node, the eight filter
// outputs summed and passed through a compander, and input 1 is the same node
// at line level.  The driver used to answer input 0 from a canned 34 byte
// waveform whose only job was to get the boot ROM's filter auto-tune out of
// the way.
//
// That auto-tune (ROM 0xf571, run for each of the eight voices) is the reason
// the feedback has to be real.  For each channel it sets the resonance to 0xff
// so that the filter oscillates, then polls the converter in a tight loop,
// roughly one read every 50 CPU cycles or 20kHz, counting reads between a rise
// past 0x90 and a fall to 0x70 or below.  It does four such cycles per trial.
// If the count is too low it lowers the cutoff CV and if too high it raises
// it, until the count lands in 129 to 132.  The cutoff byte that achieves
// that, less a nominal 0x7a, is the voice's calibration offset, which the OS
// adds to every cutoff CV afterwards.
//
// So the ROM is asking this model one question, at what CV does this chip
// oscillate at about 610Hz, and the answer is a measurement of the CV law
// above against the instrument's own calibration target.
//
// The gain from a filter output to the converter's input is not known, nor is
// the compander's law or the converter's reference, so the scale below is a
// choice: one voice at the CEM3328's nominal full swing (1.0 in this model,
// 6.5V peak to peak) reads as the converter's full scale, with silence at
// mid-scale.  What that buys is margin, which is all the ROM needs: a
// self-oscillating voice settles at about 2.26V peak to peak, which codes as
// +/-44 counts about 0x80, where the ROM's thresholds are +/-16.
constexpr double  ADC_UNITS_FULL_SCALE = 1.0;
constexpr uint8_t ADC_MID_SCALE        = 0x80;

class mirage_filters_device : public device_t, public device_sound_interface
{
public:
	mirage_filters_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// A write anywhere in 0xe400-0xe41f: the data goes into the DAC and the
	// address into the route latch.  Called from enmirage_state::coefficients_w.
	void cv_w(offs_t offset, uint8_t data);

	// The summer's output as the DOC's 8-bit converter codes it.  Brings the
	// stream up to the current instant first: the ROM's calibration reads this
	// far faster than the stream is generated, and a stale sample would make it
	// measure the emulator's buffering.  aux is whatever else is mixed into
	// that node (the audio input, which this driver stands in for with the
	// cassette), in the same units.
	uint8_t last_output(double aux = 0.0);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_post_load() override;
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	void recalc(int ch);
	double noise();

	sound_stream *m_stream = nullptr;
	uint32_t m_rate = 0;

	// Held state: the sixteen sample-and-holds and the DAC latch.  Kept in
	// DAC byte units (0-255, fractional between strobes) so that what is
	// saved is what the hardware holds, and every coefficient below is
	// derived from it.
	double m_vf[8]{};
	double m_vq[8]{};
	uint8_t m_dac = 0;

	// Filter state: four pole outputs per voice.
	double m_z[8][4]{};

	// The last sample the summer produced, in the same units as the filter
	// outputs and before SUMMER_SCALE, which is a headroom convention of this
	// model and not a divider on the board.  Taking it from this side is what
	// makes ADC_UNITS_FULL_SCALE a datasheet quantity.
	double m_last_sum = 0.0;

	// Derived from m_vf and m_vq by recalc(); not saved, rebuilt on load.
	double m_g[8]{}, m_g2[8]{}, m_g3[8]{}, m_g4[8]{};
	double m_r[8]{}, m_gain[8]{};

	// Noise generator state.  A fixed seed and a xorshift rather than the
	// machine's RNG, so that two runs of the same experiment record the same
	// output.
	uint32_t m_noise = 0x13579bdfU;
};

DEFINE_DEVICE_TYPE_PRIVATE(MIRAGE_FILTERS, mirage_filters_device, mirage_filters_device, "mirage_filters", "Mirage CEM3328 filters")

mirage_filters_device::mirage_filters_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, MIRAGE_FILTERS, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
{
}

void mirage_filters_device::device_start()
{
	// Eight in, one out: the Mirage is mono, and the eight filter outputs are
	// summed through equal resistors.  Run at whatever the machine's output
	// rate is and let the framework resample the DOC's output into it.
	m_stream = stream_alloc(8, 1, SAMPLE_RATE_OUTPUT_ADAPTIVE);

	save_item(NAME(m_vf));
	save_item(NAME(m_vq));
	save_item(NAME(m_dac));
	save_item(NAME(m_z));
	save_item(NAME(m_last_sum));
	save_item(NAME(m_noise));

	// m_rate = 0 forces the first sound_stream_update to build coefficients.
}

void mirage_filters_device::device_post_load()
{
	for (int ch = 0; ch < 8; ch++)
		recalc(ch);
}

// One white sample, uniform, at NOISE_RMS_UNITS r.m.s. (uniform on [-a, a) has
// an r.m.s. of a/sqrt(3), hence the factor).
double mirage_filters_device::noise()
{
	m_noise ^= m_noise << 13;
	m_noise ^= m_noise >> 17;
	m_noise ^= m_noise << 5;
	return double(int32_t(m_noise)) / 2147483648.0 * NOISE_RMS_UNITS * 1.7320508075688772;
}

// The CV law, and the bilinear coefficients for one voice.  Called on every
// routed DAC write and whenever the output rate changes under us.
void mirage_filters_device::recalc(int ch)
{
	if (m_rate == 0)
		return;

	const double vc = (m_vf[ch] / 255.0) * CV_V_SPAN / CV_DIVIDER + CV_OFFSET_V;
	double fc = F_ZERO_HZ * pow(2.0, vc / F_VOLTS_PER_OCTAVE);
	fc = std::clamp(fc, 1.0, FC_MAX_FRACTION * m_rate);

	// Topology-preserving one-pole: G is the prewarped integrator gain, g the
	// coefficient of a single stage.  Four identical stages.
	const double G = tan(std::numbers::pi * fc / m_rate);
	const double g = G / (1.0 + G);
	m_g[ch]  = g;
	m_g2[ch] = g * g;
	m_g3[ch] = m_g2[ch] * g;
	m_g4[ch] = m_g3[ch] * g;

	// Loop gain.  4 is the four pole oscillation threshold, and Q_OSC_BYTE is
	// the DAC byte the datasheet puts it at, so the two ends agree by
	// construction and the byte scale between them is linear, as the
	// resonance CV is.
	m_r[ch] = 4.0 * m_vq[ch] / Q_OSC_BYTE;

	// The loop's own passband droop is 1/(1+r); undo it, then apply the
	// datasheet's drop.  Doing it in that order is what makes the -12dB figure
	// mean what the datasheet means by it rather than compounding with the
	// topology's own loss.  (As a check on the model, 1/(1+r) at r = 4 is
	// -14dB, so the shape the unmodified chip gets for free and the datasheet
	// number agree to about 2dB.)
	const double rq = std::min(m_r[ch], 4.0);
	m_gain[ch] = (1.0 + m_r[ch]) * pow(10.0, PASSBAND_DROP_DB / 20.0 * rq / 4.0);
}

// Address bit 3 inhibits the cutoff selector and bit 4 the resonance
// selector, so a write with neither set reaches both.
void mirage_filters_device::cv_w(offs_t offset, uint8_t data)
{
	m_stream->update();

	m_dac = data;
	const int ch = offset & 7;
	const int fs = (offset >> 3) & 3;

	// Each strobe closes most, not all, of the gap: the hold capacitor
	// charges through the selector's on-resistance for the few microseconds
	// the route stays open.  Repeated control passes converge geometrically.
	if (fs == 0 || fs == 2)                     // 0xe400 / 0xe410: cutoff
		m_vf[ch] += CV_SETTLE_K * (double(m_dac) - m_vf[ch]);
	if (fs == 0 || fs == 1)                     // 0xe400 / 0xe408: resonance
		m_vq[ch] += CV_SETTLE_K * (double(m_dac) - m_vq[ch]);

	if (fs != 3)                                // 0xe418: preload, routed nowhere
		recalc(ch);
}

void mirage_filters_device::sound_stream_update(sound_stream &stream)
{
	if (stream.sample_rate() != m_rate)
	{
		m_rate = stream.sample_rate();
		for (int ch = 0; ch < 8; ch++)
			recalc(ch);
	}

	for (int i = 0; i < stream.samples(); i++)
	{
		double sum = 0.0;

		for (int ch = 0; ch < 8; ch++)
		{
			const double g = m_g[ch], g4 = m_g4[ch], r = m_r[ch];
			double *z = m_z[ch];

			const double x = double(stream.get(ch, i)) * DOC_CHANNEL_HEADROOM + noise();

			// What the four stages would put out on their stored state alone,
			// with no input this sample.  Having it in closed form is what
			// lets the feedback be solved rather than delayed by a sample.
			const double s = (1.0 - g) * (m_g3[ch] * z[0] + m_g2[ch] * z[1] + g * z[2] + z[3]);

			// Solve y = g^4 * (x - r * limit(y)) + s.  Below the limiter the
			// loop is linear; above it the fed-back term is a constant, so
			// both branches are exact and no iteration is needed.  Saturating
			// only ever feeds back less, so a solution past the bound stays
			// past it: the branch cannot disagree with itself.
			double y = (g4 * x + s) / (1.0 + r * g4);
			if (y > Q_LIMIT)
				y = g4 * (x - r * Q_LIMIT) + s;
			else if (y < -Q_LIMIT)
				y = g4 * (x + r * Q_LIMIT) + s;

			const double u = x - r * std::clamp(y, -Q_LIMIT, Q_LIMIT);

			// Run the stages for real to carry the state forward.  The fourth
			// output is the y solved above, by construction.
			double v = (u - z[0]) * g;   const double y1 = v + z[0];  z[0] = y1 + v;
			v = (y1 - z[1]) * g;         const double y2 = v + z[1];  z[1] = y2 + v;
			v = (y2 - z[2]) * g;         const double y3 = v + z[2];  z[2] = y3 + v;
			v = (y3 - z[3]) * g;         const double y4 = v + z[3];  z[3] = y4 + v;

			sum += y4 * m_gain[ch];
		}

		// Clamp rather than put because eight resonant voices peaking together
		// can exceed the summer's headroom, as they can on the board.
		m_last_sum = sum;
		stream.put_clamp(0, i, sum * SUMMER_SCALE, 1.0);
	}
}

uint8_t mirage_filters_device::last_output(double aux)
{
	m_stream->update();

	const double code = double(ADC_MID_SCALE)
			+ (m_last_sum + aux) / ADC_UNITS_FULL_SCALE * double(ADC_MID_SCALE);
	return uint8_t(std::clamp(code, 0.0, 255.0));
}

class enmirage_state : public driver_device
{
public:
	enmirage_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_sample_ram(*this, "sampleram", 1024 * 128, ENDIANNESS_BIG)
		, m_sample_bank(*this, "samplebank")
		, m_display(*this, "display")
		, m_fdc(*this, "wd1772")
		, m_floppy_connector(*this, "wd1772:0")
		, m_via(*this, "via6522")
		, m_irq_merge(*this, "irqmerge")
		, m_cassette(*this, "cassette")
		, m_acia(*this, "acia6850")
		, m_kbd(*this, "kbd")
		, m_filters(*this, "filters")
		, m_wheel(*this, {PITCH_TAG, MOD_TAG})
		, m_key(*this, {"pb5", "pb6", "pb7"})
	{
	}

	void mirage(machine_config &config);
	void enmirage_es5503_map(address_map &map) ATTR_COLD;

	void init_mirage();
	DECLARE_INPUT_CHANGED_MEMBER(input_changed);
	static void floppy_formats(format_registration &fr);

protected:
	virtual void machine_start() override ATTR_COLD;
	void coefficients_w(offs_t offset, uint8_t data);

private:
	void update_keypad_matrix();

	uint8_t mirage_via_read_portb();
	void mirage_via_write_porta(uint8_t data);
	void mirage_via_write_portb(uint8_t data);
	uint8_t mirage_adc_read();

	void mirage_map(address_map &map) ATTR_COLD;

	virtual void machine_reset() override ATTR_COLD;

	required_device<mc6809e_device> m_maincpu;
	memory_share_creator<uint8_t> m_sample_ram;
	required_memory_bank m_sample_bank;
	required_device<pwm_display_device> m_display;
	required_device<wd1772_device> m_fdc;
	required_device<floppy_connector> m_floppy_connector;
	required_device<via6522_device> m_via;
	required_device<input_merger_device> m_irq_merge;
	required_device<cassette_image_device> m_cassette;
	required_device<acia6850_device> m_acia;
	required_device<mirage_keyboard_device> m_kbd;
	required_device<mirage_filters_device> m_filters;

	required_ioport_array<2> m_wheel;
	required_ioport_array<3> m_key;

	int m_mux_value;
	int m_key_col_select;
};

void enmirage_state::floppy_formats(format_registration &fr)
{
	fr.add_mfm_containers();
	fr.add(FLOPPY_ESQ8IMG_FORMAT);
	fr.add(FLOPPY_HFE_FORMAT);
}

static void ensoniq_floppies(device_slot_interface &device)
{
	device.option_add("35dd", PANA_JU_363);
}

uint8_t enmirage_state::mirage_adc_read()
{
	uint8_t value = 0;
	switch(m_mux_value & 0x03)
	{
		case 0:
			/* compressed and mixed input: audio in and the ES5503 through the eight filters.
			   The compander is not modelled, so this and case 1 read alike. */
			value = m_filters->last_output(m_cassette->input());
			LOGADCREAD("%s, 5503 sample: channel: compressed input, data: $%02x\n", machine().describe_context(), value);
			break;
		case 1:
			value = m_filters->last_output(m_cassette->input()); /* line level and mixed input: audio in and ES 5503 */
			LOGADCREAD("%s, 5503 sample: channel: line input, data: $%02x\n", machine().describe_context(), value);
			break;
		case 2:
			value = m_wheel[0]->read(); /* pitch wheel */
			LOGADCREAD("%s, 5503 sample: channel: pitch wheel, data: $%02x\n", machine().describe_context(), value);
			break;
		case 3:
			value = m_wheel[1]->read(); /* mod wheel */
			LOGADCREAD("%s, 5503 sample: channel: mod wheel, data: $%02x\n", machine().describe_context(), value);
			break;
	}

	return value;
}

void enmirage_state::machine_start()
{
	save_item(NAME(m_mux_value));
	save_item(NAME(m_key_col_select));
	m_sample_bank->configure_entries(0, 4, m_sample_ram, 0x8000);
}

void enmirage_state::machine_reset()
{
	m_sample_bank->set_entry(0);
	m_mux_value = 0;
}

void enmirage_state::mirage_map(address_map &map)
{
	map(0x0000, 0x7fff).bankrw("samplebank"); // 32k window on 128k of sample RAM
	map(0x8000, 0xbfff).ram(); // main RAM
	map(0xc000, 0xdfff).ram(); // expansion RAM
	map(0xe100, 0xe101).rw("acia6850", FUNC(acia6850_device::read), FUNC(acia6850_device::write));
	map(0xe200, 0xe2ff).m(m_via, FUNC(via6522_device::map));
	map(0xe400, 0xe41f).w(FUNC(enmirage_state::coefficients_w));
	map(0xe800, 0xe803).rw(m_fdc, FUNC(wd1772_device::read), FUNC(wd1772_device::write));
	map(0xec00, 0xecef).rw("es5503", FUNC(es5503_device::read), FUNC(es5503_device::write));
	map(0xf000, 0xffff).rom().region("osrom", 0);
}

void enmirage_state::coefficients_w(offs_t offset, uint8_t data)
{
	uint8_t channel = offset & 0x07;
	uint8_t filter_input = (offset >> 3) & 0x03;

	LOGFILTERWRITE("%s, filter update: channel: %d, data: $%02x (%s%s%s%s)\n",
				machine().describe_context(),
				channel,
				data,
				(filter_input & 0x01) == 0 ? "VF" : "", /* cut-off frequency */
				(filter_input & 0x03) == 0 ? " and " : "",
				(filter_input & 0x02) == 0 ? "VQ" : "", /* filter resonance */
				(filter_input & 0x03) == 0x03 ? "preload dac" : "");

	// one DAC with a latched route, so the filters decode the raw offset
	m_filters->cv_w(offset, data);
}

// port A:
//  bits 5/6/7 keypad rows 0/1/2 return
INPUT_CHANGED_MEMBER(enmirage_state::input_changed)
{
	update_keypad_matrix();
}

void enmirage_state::update_keypad_matrix()
{
	uint8_t value;

	value  = ((m_key[0]->read() >> m_key_col_select) & 0x01) << 5;
	value |= ((m_key[1]->read() >> m_key_col_select) & 0x01) << 6;
	value |= ((m_key[2]->read() >> m_key_col_select) & 0x01) << 7;

	m_via->write_pa(value);
}

// port B:
//  bit 6: IN disk load
//  bit 5: IN Q Chip sync

uint8_t enmirage_state::mirage_via_read_portb()
{
	uint8_t value = m_via->read_pb();

	floppy_image_device *floppy = m_floppy_connector ? m_floppy_connector->get_device() : nullptr;
	if (floppy)
	{
		if (floppy->dskchg_r())
			value |= 0x40;
		else
			value &= ~0x40;
	}

	return value;
}

// port A: front panel
// bits 0/1/2: dual purpose (0 to 7 lines, though a 74LS145 decoder):
//      keyboard matrix column select
//      7 segment display driver
//  bits 3/4 = right and left 7 segment display enable
//  bits 5/6/7 = Keyboard matrix row sense from 0 to 2
void enmirage_state::mirage_via_write_porta(uint8_t data)
{
	u8 segdata = data & 7;
	m_display->matrix(((data >> 3) & 3) ^ 3, (1<<segdata));

	uint8_t new_select = (data & 0x07);
	if (m_key_col_select != new_select)
	{
		m_key_col_select = new_select;
		update_keypad_matrix();
	}
}

// port B:
//  bit 7: OUT UART clock
//  bit 4: OUT disk select, motor on (it does not reset the keyboard controller)
//  bit 3: OUT sample/play
//  bit 2: OUT mic line/in
//  bit 1: OUT upper/lower bank (64k halves)
//  bit 0: OUT bank 0/bank 1 (32k quarters)

void enmirage_state::mirage_via_write_portb(uint8_t data)
{
	int bank = 0;

	// handle sound RAM bank switching
	bank = data & 0x03;
	m_sample_bank->set_entry(bank);

	// handle floppy motor on
	floppy_image_device *floppy = m_floppy_connector->get_device();
	if (floppy)
		floppy->mon_w(data & 0x10 ? 1 : 0);

	// The keyboard controller's reset comes from the board's power-on reset circuit, not from this port.

	// record audio input mixer position
	m_mux_value = (data >> 2) & 0x03;

	// handle acia clock
	// this bit is set by the internal via timer
	int clock = (data >> 7) & 0x01;
	m_acia->write_txc(clock);
	m_acia->write_rxc(clock);
}

void enmirage_state::enmirage_es5503_map(address_map &map)
{
	map(0x00000, 0x1ffff).ram().share("sampleram");
}

void enmirage_state::mirage(machine_config &config)
{
	// The 6809E's E clock is 1 MHz.  The DOC's CLKIN is 8 MHz (half of the
	// board's 16 MHz crystal) and the DOC divides that by eight to produce E,
	// so the CPU is a genuine 6809E taking E as an input and cannot be running
	// from its own oscillator.
	MC6809E(config, m_maincpu, 1000000);
	m_maincpu->set_addrmap(AS_PROGRAM, &enmirage_state::mirage_map);

	INPUT_MERGER_ANY_HIGH(config, m_irq_merge).output_handler().set_inputline(m_maincpu, M6809_IRQ_LINE);
	// <0> via6522
	// <1> wd1772
	// <2> es5502
	// <3> cartridge connector (TODO)

	SPEAKER(config, "speaker").front_center();

	CASSETTE(config, m_cassette);
	m_cassette->set_default_state(CASSETTE_PLAY | CASSETTE_MOTOR_DISABLED | CASSETTE_SPEAKER_ENABLED);
	m_cassette->add_route(ALL_OUTPUTS, "speaker", 1.0);

	es5503_device &es5503(ES5503(config, "es5503", 8000000));
	es5503.set_channels(8);
	es5503.set_addrmap(0, &enmirage_state::enmirage_es5503_map);
	es5503.irq_func().set(m_irq_merge, FUNC(input_merger_device::in_w<2>));
	es5503.adc_func().set(FUNC(enmirage_state::mirage_adc_read));

	// the DOC's channel n output is the input of the filter for voice n
	MIRAGE_FILTERS(config, m_filters);
	for (int i = 0; i < 8; i++)
		es5503.add_route(i, m_filters, 1.0, i);
	m_filters->add_route(0, "speaker", 1.0);

	// The VIA runs at 2 MHz, which is not E.  The OS generates the ACIA's
	// 500 kHz clock (31250 baud with the divide-by-16 mode) on PB7 using a timer
	// 1 latch of zero, a PB7 period of four VIA clocks.  The VIA's phi2 input is
	// wired to the DRAM /CAS strobe rather than E, and /CAS strobes once for
	// each of the two bus masters (the CPU and the DOC) in every system clock
	// period, so it runs at twice E.
	MOS6522(config, m_via, 2000000);
	// The OS clocks the ACIA from PB7 (see above), which only toggles for a zero timer 1
	// latch if the VIA is told to.
	m_via->set_t1_zero_latch_toggles_pb7(true);
	m_via->writepa_handler().set(FUNC(enmirage_state::mirage_via_write_porta));
	m_via->readpb_handler().set(FUNC(enmirage_state::mirage_via_read_portb));
	m_via->writepb_handler().set(FUNC(enmirage_state::mirage_via_write_portb));
	m_via->irq_handler().set(m_irq_merge, FUNC(input_merger_device::in_w<0>));

	// the keyboard controller stand-in on the VIA shift register
	MIRAGE_KEYBOARD(config, m_kbd, 0);
	m_kbd->sclk_handler().set(m_via, FUNC(via6522_device::write_cb1));
	m_kbd->sdata_handler().set(m_via, FUNC(via6522_device::write_cb2));
	m_via->ca2_handler().set(m_kbd, FUNC(mirage_keyboard_device::sack_w));

	PWM_DISPLAY(config, m_display).set_size(2, 8);
	config.set_default_layout(layout_enmirage);

	ACIA6850(config, m_acia).txd_handler().set("mdout", FUNC(midi_port_device::write_txd));
	m_acia->irq_handler().set_inputline(m_maincpu, M6809_FIRQ_LINE);
	MIDI_PORT(config, "mdin", midiin_slot, "midiin").rxd_handler().set(m_acia, FUNC(acia6850_device::write_rxd));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");

	WD1772(config, m_fdc, 8000000);
	m_fdc->intrq_wr_callback().set_inputline(m_maincpu, INPUT_LINE_NMI);
	m_fdc->drq_wr_callback().set(m_irq_merge, FUNC(input_merger_device::in_w<1>));

	FLOPPY_CONNECTOR(config, "wd1772:0", ensoniq_floppies, "35dd", enmirage_state::floppy_formats).enable_sound(true);

	// This clock allows the CPU to keep in sync with the sound chip - may not be used in the firmware
	clock_device &es5503_ca3_clock(CLOCK(config, "ca3_clock", XTAL(8'000'000) / 16));
	es5503_ca3_clock.signal_handler().set(m_via, FUNC(via6522_device::write_pb5));
}

static INPUT_PORTS_START(mirage)
	PORT_START("pb5") /* KEY ROW 0 */
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Load Upper")         PORT_CODE(KEYCODE_A) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Load Lower")         PORT_CODE(KEYCODE_B) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Sample Upper")       PORT_CODE(KEYCODE_C) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Play Sequence")      PORT_CODE(KEYCODE_D) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Load Sequence")      PORT_CODE(KEYCODE_E) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Save Sequence")      PORT_CODE(KEYCODE_F) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Record Sequence")    PORT_CODE(KEYCODE_G) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Sample Lower")       PORT_CODE(KEYCODE_H) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_START("pb6") /* KEY ROW 1 */
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3")      PORT_CODE(KEYCODE_3)        PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6")      PORT_CODE(KEYCODE_6)        PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9")      PORT_CODE(KEYCODE_9)        PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5")      PORT_CODE(KEYCODE_5)        PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8")      PORT_CODE(KEYCODE_8)        PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0/Prog") PORT_CODE(KEYCODE_0)        PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2")      PORT_CODE(KEYCODE_2)        PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Enter")  PORT_CODE(KEYCODE_ENTER)    PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_START("pb7") /* KEY ROW 2 */
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1")          PORT_CODE(KEYCODE_1)    PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4")          PORT_CODE(KEYCODE_4)    PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7")          PORT_CODE(KEYCODE_7)    PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("On/Up")      PORT_CODE(KEYCODE_UP)   PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Param")      PORT_CODE(KEYCODE_I)    PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Off/Down")   PORT_CODE(KEYCODE_DOWN) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Value")      PORT_CODE(KEYCODE_J)    PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Cancel")     PORT_CODE(KEYCODE_K)    PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(enmirage_state::input_changed), 0)

	PORT_START(PITCH_TAG)
	PORT_BIT(0xff, 0x7f, IPT_PADDLE) PORT_NAME("Pitch Wheel") PORT_SENSITIVITY(100) PORT_KEYDELTA(10) PORT_MINMAX(0x00,0xff) PORT_CODE_INC(KEYCODE_4_PAD) PORT_CODE_DEC(KEYCODE_1_PAD) PORT_PLAYER(1)
	PORT_START(MOD_TAG)
	PORT_BIT(0xff, 0x7f, IPT_PADDLE) PORT_NAME("Mod Wheel") PORT_SENSITIVITY(100) PORT_KEYDELTA(10) PORT_MINMAX(0x00,0xff) PORT_CODE_INC(KEYCODE_6_PAD) PORT_CODE_DEC(KEYCODE_3_PAD) PORT_PLAYER(1)
INPUT_PORTS_END

ROM_START(enmirage)
	ROM_REGION(0x1000, "osrom", 0)
	ROM_LOAD("mirage.bin", 0x0000, 0x1000, CRC(9fc7553c) SHA1(ec6ea5613eeafd21d8f3a7431a35a6ff16eed56d))

	ROM_REGION(0x20000, "es5503", ROMREGION_ERASE)
ROM_END

void enmirage_state::init_mirage()
{
	floppy_image_device *floppy = m_floppy_connector ? m_floppy_connector->get_device() : nullptr;
	if (floppy)
	{
		m_fdc->set_floppy(floppy);
	}
}

} // anonymous namespace


CONS(1984, enmirage, 0, 0, mirage, mirage, enmirage_state, init_mirage, "Ensoniq", "Mirage DMS-8", MACHINE_NOT_WORKING)
