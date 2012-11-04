/**
 * Project: Simon MacDonald
 * Version: 1.0
 * Author: Tom Price
 * Date: October 2012
 * License: Creative Commons
 * 
 * Simon MacDonald is a clone of the 1970s era electronic memory game "Simon" 
 * by MB Electronics, mashed inside a Fisher Price Little People Animal Sounds Farm.
 * Instead of following a sequence of beeps, the player has to recreate a sequence
 * of animal noises by pressing the appropriate switches on the farm toy.
 *
 * Simon MacDonald runs on an AVR Attiny2313 with the default fuse settings.
 * For more information see (blog link) (github link)
 *
 * Description of original Simon game:
 * http://bit.ly/XH4Wrn
 *
 * Fisher Price Little People Animal Sounds Farm:
 * http://amzn.to/TK02FR
 */
  
/**
 * Description of game implementation:
 * 
 * I have tried to preserve the main features of the original Simon game as described here:
 * http://bit.ly/XH4Wrn
 *
 * The main adaptations that I have made are;
 * (1) Not to implement the multiplayer games
 * (2) To have intermediate fanfares after 8, 14,and 20 moves instead of preset skill levels
 * (3) To slow the display sequence initially to help young children and so that the sounds 
 *     can be heard
 * (4) To allow extra time for user input to reach the various switches
 *
 * The Little People Animal Sounds Farm is a large toy with a detachable base. 
 * Inside the base are 6 switches governing 6 noises (OINK, BAA, MOO, NEIGH, CROW, TUNE)
 * which are played back by a PCB featuring a blob shaped chip. 
 *
 * There is ample space inside the base to fit a circuit board with a microcontroller. 
 * In addition to the microcontroller, I added a slide switch to select between the original
 * hit-switch-make-noise mode and the Simon game, and 4 LEDs next to the animal switches.
 *
 * I used the AVR Attiny2313 because of its low cost, and because its relatively large 
 * number of pins means that the inputs (switches) and outputs (to LEDs and sound chip) 
 * do not need to be multiplexed.
 *
 * The game is implemented as a Finite State Machine. In START_SIMON mode the microcontroller  
 * initializes the random move vector. Next is LISTEN mode in which the sequence is displayed.
 * REPEAT mode follows which seeks input from the user by way of the switches. Incorrect
 * responses lead to LOSE mode. Correct responses lead back to LISTEN mode with a longer sequence
 * unless the maximum length sequence is reached, in which case WIN mode follows. WIN and LOSE
 * modes both lead back to START_SIMON mode. Moving the slide switch in REPEAT mode leads back to FARM
 * mode.
 *
 * Interrupt-based timing routines are used to debounce the switches and implement delays of
 * various kinds. The microcontroller is allowed to idle between interrupts, thereby saving
 * power. Pin change interrupts are used in preference to polling the switches.
 *
 * Pseudorandom sequences of moves are generated using a Galois Linear Feedback Shift Register
 * (http://en.wikipedia.org/wiki/Linear_feedback_shift_register).
 * 
 * Making the noises involves a bit of trickery. The animal noises are too long for quick play
 * and need to be truncated. An animal noise can be curtailed by sounding a different animal noise.
 * The problem comes when sounding the same noise twice in a row. This can be done by sounding a
 * different intermediate noise very quickly.
 */o
 
/**
 * Compilation and uploading:
 *
 * I compiled using avr-gcc with gcc version 4.5.3 and the -Os optimization flag. See the Makefile.
 * I uploaded the hex file using avrdude version 5.11.1 and a selfmade USBasp after yuki-lab.jp
 * The Attiny2313 chip used the default fuses -U lfuse:w:0x64:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
 * and the internal oscillator divided down to 1 Mhz.
 */
 
/******************************************************
 *                                                    *
 *                 D E F I N E S                      *
 *                                                    *
 ******************************************************/
 
/**
 * define output pins to make noises
 * all are active high outputs
 */
#define NOISE_PORT  PORTD
#define OINK_NOISE  PD0
#define BAA_NOISE   PD1
#define MOO_NOISE   PD2
#define NEIGH_NOISE PD3
#define CROW_NOISE  PD4
#define TUNE_NOISE  PD5

/**
 * define input pins for switches
 * all are active high inputs with 10K pulldown resistors
 */
#define SWITCH_PORT  PINB
#define OINK_SWITCH  PB0
#define BAA_SWITCH   PB1
#define MOO_SWITCH   PB2
#define NEIGH_SWITCH PB3
#define CROW_SWITCH  PB4
#define TUNE_SWITCH  PB5
#define MODE_SWITCH  PB6

/**
 * define output ports and pins for LEDs
 */
#define OINK_LED_PORT  PORTD
#define BAA_LED_PORT   PORTA
#define MOO_LED_PORT   PORTB
#define NEIGH_LED_PORT PORTA
#define OINK_LED  PD6
#define BAA_LED   PA0
#define MOO_LED   PB7
#define NEIGH_LED PA1

/**
 * define move values corresponding to:
 * the output pins on PORTB for making noises
 * the input pins on PORTD for switches
 * OINK, BAA, MOO, NEIGH are the 4 possible moves
 * CROW and TUNE are sounded during the WIN and LOSE sequences respectively
 */
#define OINK  0
#define BAA   1
#define MOO   2
#define NEIGH 3
#define CROW  4
#define TUNE  5

// Maximum number of moves
#define MAX_NUM_MOVES  32   

// Display timing constants
// slightly slower display than the original game
#define DISPLAY_DURATION_LONG_MS	1200
#define DISPLAY_DURATION_MIDDLING_MS	600
#define DISPLAY_DURATION_SHORT_MS	400
#define DISPLAY_GAP_MS	50
#define DELAY_AFTER_LOSS_MS	500
#define DELAY_AFTER_REPEAT_MS	1000
#define DELAY_BETWEEN_GAMES_MS	2000

// debounce time
#define DEBOUNCE_MS 80

// response time limit
// 5 s instead of the original 3 s
#define RESPONSE_MS 5000

// led hold time after player move
#define HOLD_LED_MS	1000 

// noise pulse time
// set to 0  to turn all noises off
// set to 20 otherwise
#define HOLD_NOISE_MS	20

// LFSR seed (>0) 
#define LFSR_SEED 12345

// Default 1 MHz internal Oscillator
#define F_CPU 1000000 

// Debug flag
// Comment out to remove DEBUG code: saves 100 bytes or so
#define DEBUG 1
  
/******************************************************
 *                                                    *
 *                 I N C L U D E S                    *
 *                                                    *
 ******************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
/*
#include <avr/wdt.h>
#include <avr/power.h>
*/

/******************************************************
 *                                                    *
 *     V A R I A B L E   D E C L A R A T I O N s      *
 *                                                    *
 ******************************************************/

/**
 * enum Mode: START_FARM, FARM, START_SIMON, LISTEN, REPEAT, WIN, LOSE
 *  
 * \brief Define modes that define the game flow.
 *
 * START_FARM: A short initiation sequence before moving into FARM mode.
 *         
 * FARM: This is the original state of the farmhouse toy. Wait for input from  
 *       switches. When a switch is turned on the relevant noise is sounded
 *       (oink, baa, moo, neigh, crow, tune). Meanwhile, continuously cycle 
 *       through the pseudorandom number sequence. A slide switch selects between 
 *       FARM and START_SIMON mode at any time. Every minute or so the LEDs blink.
 *
 * START_SIMON: Flash LEDs briefly, then use the current state of the pseudorandom number
 *        generator to create a sequence of 2-bit pseudorandom numbers which are
 *        stored in binary form in the moves[] array. Then the MCU moves forward
 *        to LISTEN mode. Input from the player is ignored.
 *
 * LISTEN: The microcontroller (Simon) displays the current sequence of moves to be
 *        repeated by the player, then moves to REPEAT mode. Input from the player is
 *        ignored.
 * 
 * REPEAT: The player's turn. The MCU waits for a sequence of inputs corresponding to the 
 *           current number of moves in the current_round. If the player enters an incorrect 
 *           sequence or times out the game goes to LOSE mode. Otherwise the number of  
 *           moves in the current_round is incremented. If the number of moves exceeds 
 *           MAX_NUM_MOVES the game moves to WIN mode, otherwise it returns to LISTEN mode.
 *
 * WIN: Play tune and rhythmically flash LEDs before returning to START_SIMON.
 *
 * LOSE: Crow and return to START_SIMON.
 */
 
/**
 * \var  Mode  current_state  Records the current status of the game flow.
 *
 * \var  uint8_t  buttons_pressed  Records which switches are activated.
 *
 * \var  uint8_t  previous_buttons_pressed  Records which switches were previously activated.
 *
 * \var  uint16_t  delay_counter  millisecond delay timer.
 *
 * \var  uint16_t  debounce_counter  debounce timer.
 *
 * \var  uint16_t  response_counter  response timer.
 *
 * \var  uint16_t   moves  Records the moves which the player must match.
 * 
 * \var  uint16_t  *move  Pointer to current move (2 bits)
 *
 * \var  uint16_t  random  Current status of the LFSR.
 * 
 * \var  uint8_t  current_round  Counts the number of moves Simon has displayed 
 *                            and that the player must match.
 * 
 * \var  uint8_t  waiting  Flag to indicate if player move is still awaited.
 * 
 * \var  uint8_t  last_noise  The last noise that was made.
 * 
 * \var  uint8_t  next_noise  The next noise to be made after the current noise terminates.
 * 
 * \var  uint8_t  next_noise_hold  How long to hold the next noise in milliseconds.
 */ 
 
// volatile variable declarations
volatile enum Mode 
{
	START_FARM,
	FARM,
	START_SIMON,
	LISTEN,
	REPEAT,
	WIN,
	LOSE
} current_state;
volatile uint8_t buttons_pressed = 0;
volatile uint8_t previous_buttons_pressed = 0;
volatile uint16_t delay_counter = 0;
volatile uint16_t debounce_counter = 0;
volatile uint16_t response_counter = 0;
volatile uint16_t led_hold_counter = 0;
volatile uint8_t noise_hold_counter = 0;

// static variable declarations
static uint16_t moves[ MAX_NUM_MOVES >> 3 ]; // 8x 2-bit moves per 2-byte uint16_t 
static uint16_t move;
static uint16_t random = LFSR_SEED;
static uint8_t current_round; 
static uint8_t waiting;
static uint8_t last_noise = 0xFF;
static uint8_t next_noise = 0;
static uint8_t next_noise_hold = 0;

/******************************************************
 *                                                    *
 *     F U N C T I O N   D E C L A R A T I O N s      *
 *                                                    *
 ******************************************************/

void delay_ms( uint16_t delay_time );
uint16_t rand_lfsr( uint16_t lfsr );
void get_simon_move( uint8_t current_round, uint16_t *pmove );
void make_noise( uint8_t noise );
void make_switched_noise( void );
void stop_noises( void );
void set_led( uint8_t move, uint16_t hold_time, uint8_t clear );
void wait_on_leds( void );
void led_fanfare( uint8_t flashes, uint16_t hold_time );
void clear_leds( void );
void blink_leds( void );
void port_init( void );
void config( void );
void enable_input( void );
void disable_input( void );
void get_input( void );
uint8_t start_farm_mode( void );
uint8_t farm_mode( void );
uint8_t start_simon_mode( void );
uint8_t simon_mode( void );
uint8_t listen_mode( void );
uint8_t parse_player_input( void );
uint8_t get_player_move( void );
uint8_t repeat_mode( void );
uint8_t lose_mode( void );
uint8_t win_mode( void );

#ifdef DEBUG
void set_switched_led( void ); 
#endif
 
/******************************************************
 *                                                    *
 *               S U B R O U T I N E S                *
 *                                                    *
 ******************************************************/

/**
 * delay_ms()
 *
 * \param   uint16_t  delay_time  Delay time in milliseconds
 *
 * \brief Millisecond delay with idle
 */
void delay_ms( uint16_t delay_time )
{
    delay_counter = delay_time;
	do
	{
		sleep_mode();
	}
	while( delay_counter );
}

/**
 * rand_lfsr()
 *
 * \param   uint16_t  lfsr  Current value of the lfsr
 *
 * \brief Generate pseudorandom number using Galois Linear Feedback Shift Register
 * http://en.wikipedia.org/wiki/Linear_feedback_shift_register
 * Needs initial seed value.
 */
uint16_t rand_lfsr( uint16_t lfsr )
{
	return ( lfsr >> 1 ) ^ ( -( lfsr & (uint16_t) 1 ) & (uint16_t) 0xB400 );
}

/**
 * get_simon_move()
 *
 * \param   uint8_t  current_round  Number between 0 and MAX_NUM_MOVES identifying current_round
 *
 * \param   uint16_t  *move  Pointer to variable in which 2 bit move will be stored
 *
 * \var  uint8_t  j  Temporary variable
 * 
 * \brief Retrieve move for current current_round
 */
void get_simon_move( uint8_t current_round, uint16_t *pmove )
{
	uint8_t j = ( current_round & 7 ) << 1;
	*pmove = ( moves[ current_round >> 3 ] & ( 3 << j ) ) >> j;
}

/**
 * stop_noises()
 *
 * \brief Send animal noise signals low. NB this does not actually turn the noises off.
 */
void stop_noises()
{
	NOISE_PORT &= 0xC0;
}

/**
 * make_noise()
 *
 * \param   uint8_t  noise  Number between 0 and 5 identifies which noise to make.
 *
 * \brief Turn on a specified animal noise.
 */
void make_noise( uint8_t noise )
{
	stop_noises(); 
	// test to see if this noise is the same as the last noise made
	if ( noise == last_noise )
	{
		// presound MOO or NEIGH for a very brief duration
		// this needs debugging - doesn't seem to work
		//stop_noises();
		if ( noise == MOO )
		{
			NOISE_PORT |= ( 1 << NEIGH );
		} else
		{
			NOISE_PORT |= ( 1 << MOO );
		} 
		noise_hold_counter = HOLD_NOISE_MS;
		next_noise = noise;
		next_noise_hold = HOLD_NOISE_MS;
	} else
	{
		NOISE_PORT |= ( 1 << noise );
		noise_hold_counter = HOLD_NOISE_MS;
		next_noise = 0xFF;
		next_noise_hold = 0;
		last_noise = noise;
	}
}

/**
 * make_switched_noise()
 *
 * \param   uint8_t  noise  "One hot" number between 0b00000001 and 0b00010000 identifies which noise to make.
 *
 * \brief Turn on animal noise corresponding to the switches turned on.
 */
void make_switched_noise()
{
	//uint8_t noises = buttons_pressed & (uint8_t) ~previous_buttons_pressed;
	uint8_t i;
	for( i = OINK_SWITCH; i <= TUNE_SWITCH; i++ )
	{
		if ( buttons_pressed & ( 1 << i ) )
		{
			make_noise( i );
			break;
		}
	}
}

/**
 * clear_leds()
 *
 * \brief Turn off all LEDs.
 */
void clear_leds()
{
	OINK_LED_PORT  &= (uint8_t) ~( 1 << OINK_LED  );
	BAA_LED_PORT   &= (uint8_t) ~( 1 << BAA_LED   );
	MOO_LED_PORT   &= (uint8_t) ~( 1 << MOO_LED   );
	NEIGH_LED_PORT &= (uint8_t) ~( 1 << NEIGH_LED );
	led_hold_counter = 0;
}

/**
 * set_led()
 *
 * \param   uint8_t  move  Which of 4 LEDs to light (0-3).
 *
 * \param   uint8_t  hold_time  How many milliseconds to hold the LED on.
 *
 * \param   uint8_t  clear  0 = do not clear LEDs, else = clear LEDs.
 *
 * \brief Turn on LED corresponding to 2-bit move.
 */
void set_led( uint8_t move, uint16_t hold_time, uint8_t clear )
{
	if ( clear )
	{
		clear_leds();
	}
	switch ( move ) 
	{
	case OINK:
	case BAA:
	case MOO:
	case NEIGH:
		led_hold_counter = hold_time;
		break;
	default:
		break;
	}
	switch ( move ) 
	{
	case OINK:
		OINK_LED_PORT  |= ( 1 << OINK_LED  );
		break;
	case BAA:
		BAA_LED_PORT   |= ( 1 << BAA_LED   );
		break;
	case MOO:
		MOO_LED_PORT   |= ( 1 << MOO_LED   );
		break;
	case NEIGH:
		NEIGH_LED_PORT |= ( 1 << NEIGH_LED );
		break;
	default:
		break;
	}
}

/**
 * wait_on_leds()
 *
 * \brief Delay until the LED(s) currently lit has turned off.
 */
void wait_on_leds()
{
	while ( led_hold_counter ) 
	{
		// idle until next interrupt
		sleep_mode();	
	};
}

#ifdef DEBUG
/**
 * set_switched_led()
 *
 * \brief Turn on LED corresponding to the switches turned on.
 */
void set_switched_led()
{
	uint8_t i;
	//uint8_t leds = buttons_pressed & (uint8_t) ~previous_buttons_pressed;
	for( i = OINK_SWITCH; i <= NEIGH_SWITCH; i++ )
	{
		if ( buttons_pressed & ( 1 << i ) ) 
		{
			set_led( i, 1000, 0 );
		}
	}
}
#endif

/**
 * led_fanfare()
 *
 * \param   uint8_t  flashes  Length of flashing LED sequence.
 * 
 * \param   uint16_t  hold_time  How long each LED is flashed for (ms).
 *
 * \brief Flash LEDs in chasing sequence.
 */
void led_fanfare( uint8_t flashes, uint16_t hold_time )
{
	uint8_t i = OINK, j, up = 1;
	for ( j = 0; j < flashes; j++ )
	{
		set_led( i, hold_time, 1 );
		wait_on_leds();
		i += up;
		if( i == OINK || i == NEIGH )
		{
			up = -up;
		}
	}
}

/**
 * blink_leds()
 * 
 * \brief Briefly blink LEDs.
 */
void blink_leds() 
{
	led_fanfare( 25, 1 );
}

/**
 * port_init()
 *
 * \brief Initialize IO ports
 */
void port_init()
{
	// set up PORTA, PORTD, and PB7 to be outputs
	// set up the rest of PORTB to be inputs
	DDRA = 0x03;
	DDRB = 0x80;
	DDRD = 0x7F;
	
	// everything off, disable internal pullup resistors
	PORTA = 0x00;
	PORTB = 0x00;
	PORTD = 0x00;
}

/**
 * config()
 *
 * \brief Initialize interrupts and other machine settings.
 */
void config( void )
{
	// set sleep mode = idle (which is the default in any case)
	set_sleep_mode( SLEEP_MODE_IDLE );
	
	// power down analog comparator
	// saves power in idle mode
	ACSR &= ~( 1 << ACD );
	
	// initialize interrupts
	// disable global interrupts
	cli();
	
	// initialize timer 0
	TCCR0B |= ( 1 << WGM02 ); // Configure timer 0 (8 bit) for CTC mode 
	TCCR0B |= ( 1 << CS01 ); // Start timer at Fcpu/8
	OCR0A = 125; // Set CTC compare value to 1kHz at 1MHz AVR clock, with a prescaler of 8 
	
	// enable CTC interrupts
	TIMSK |= ( 1 << OCIE0A );
	
	// pin change interrupts on PCINT0-PCINT6
	// any logical change triggers interrupt
	PCMSK = 0x7F;	
	
	// enable global interrupts	
	sei(); 	
}

/**
 * enable_input()
 *
 * \brief Enable pin change interrupts to capture switch changes.
 */
void enable_input()
{
	// clear record of switches pressed
	//buttons_pressed = 0;
	// enable pin change interrupts
	GIMSK |= ( 1 << PCIE );
}

/**
 * disable_input()
 *
 * \brief Disable pin change interrupts to ignore switch changes.
 */
void disable_input()
{
	// clear record of switch presses
	//previous_buttons_pressed = 0;
	//buttons_pressed = 0;
	// disable pin change interrupts
	GIMSK &= (uint8_t) ~( 1 << PCIE );
}

/**
 * get_input()
 *
 * \brief Record switch values.
 */
void get_input()
{
	// record input from switches OR'd with flag bit
	buttons_pressed = SWITCH_PORT | 0x80;
}

/**
 * start_farm_mode()
 *
 * \brief Brief introduction to farm mode. Returns FARM Mode.
 */
uint8_t start_farm_mode()
{
	// reset input
	buttons_pressed = 0;
	previous_buttons_pressed = 0;
	enable_input();
	// slow ripple of LEDs to indicate we are entering FARM mode
	led_fanfare( 7, 100 );
	return FARM;
}

/**
 * farm_mode()
 *
 * \brief Play farm toy in original mode. Returns current Mode.
 */
uint8_t farm_mode()
{
	// continuously cycle through the pseudorandom number
	// sequence while awaiting player input
	random = rand_lfsr( random );
	// blink LEDs every minute or so
	// as a reminder to turn the toy off
	if ( random == 0x0001 )
	{
		blink_leds();
	}
	if ( buttons_pressed && !debounce_counter )
	{
		disable_input();
		// if keys pressed and debounce time elapsed
		// get input from switches
		get_input();
		if ( buttons_pressed & ( 1 << MODE_SWITCH ) )
		{
			// if mode switch pressed then start Simon game
			return START_SIMON;
		} else if ( buttons_pressed ^ previous_buttons_pressed )
		{
			// switches have recently changed
			stop_noises();
			clear_leds();
			// make the appropriate noise
			make_switched_noise();
			// if DEBUG light up LEDs too
			
#ifdef DEBUG			
set_switched_led();
#endif

			// save record of button presses
			previous_buttons_pressed = buttons_pressed; 			
		} 
		enable_input();
	}
	return FARM;
}

/**
 * start_simon_mode()
 *
 * \brief Start Simon game. Returns current game Mode.
 */
uint8_t start_simon_mode()
{
	// get a next random number from sequence 
	uint8_t i;
	for( i = 0; i < ( MAX_NUM_MOVES >> 3 ); i++ )
	{
		random = rand_lfsr( random );
		moves[ i ] = random;
	}
	// ripple LEDs briefly
	led_fanfare( 25, 50 );
	// reset current_round counter
	current_round = 0;
	// reset input
	buttons_pressed = 0;
	previous_buttons_pressed = 0;
	disable_input();
	// begin first current_round
	return LISTEN;
}

/**
 * listen_mode()
 * 
 * \var  uint16_t  display_duration_ms  Display duration for current current_round in milliseconds.
 *
 * \brief Display current current_round of Simon game. Returns current game Mode.
 */
uint8_t listen_mode()
{
	uint8_t i; // current move number
	for( i = 0; i < current_round; i++ ) 
	{
		if ( i > 0 )
		{
			delay_ms( DISPLAY_GAP_MS );
		}
		// get current move (2 bits)
		get_simon_move( i, &move );
		// display move 
		uint16_t display_duration_ms;
		// duration of display gets shorter as the game goes on
		if ( current_round <= 5 )
		{
			display_duration_ms = DISPLAY_DURATION_LONG_MS;  
		} else if ( current_round <= 13 )
		{
			display_duration_ms = DISPLAY_DURATION_MIDDLING_MS; 
		} else 
		{
			display_duration_ms = DISPLAY_DURATION_SHORT_MS; 
		} 
		set_led( (uint8_t) move, display_duration_ms, 1 ); 
		make_noise( (uint8_t) move );
		wait_on_leds();
	}
	// player's turn now
	enable_input();
	return REPEAT;
}

/**
 * parse_player_input()
 *
 * \brief Make sense of player input in current current_round of Simon game. Returns current game Mode.
 */
uint8_t parse_player_input()
{
	if ( !( buttons_pressed & ( 1 << MODE_SWITCH ) ) )
	{
		// if mode switch switched back then enter START_FARM mode
		enable_input();
		return START_FARM;
	} else if ( buttons_pressed & ( 1 << CROW_SWITCH | 1 << TUNE_SWITCH ) ) 
	{	
		// if CROW or TUNE sounded then lose (harsh but fair!)
		make_switched_noise();
		return LOSE;
	}
	else if ( buttons_pressed & 0x0F )
	{
		// one or more switches on
		// make the appropriate noise
		make_switched_noise();
		// light the appropriate LED
		// parse buttons pressed for player move
		uint8_t player_move = 0xFF;
		uint8_t j; // parsed player move
		for ( j = OINK_SWITCH; j <= NEIGH_SWITCH; j++ )
		{
			if ( buttons_pressed & ( 1 << j ) )
			{
				// light the appropriate LED
				set_led( j, 1000, 1 );
				if ( player_move == 0xFF )
				{
					player_move = j;
				} else 
				{
					// more than 1 button pressed loses
					return LOSE;					
				}
			}			
		}
		// have valid player move
		if ( player_move == (uint8_t) move ) 
		{
			// correct player move
			// set flag to move to next current_round
			waiting = 0;
			return REPEAT;
		} else if ( player_move != 0xFF )
		{
			// incorrect player move
			return LOSE;
		}
	} 
	// no switch closed
	// any change must have been closing an open switch
	stop_noises();
	clear_leds();
	// keep waiting
	/* waiting = 1; */
	enable_input();		
	return REPEAT;
}

/**
 * get_player_move()
 *
 * \brief Obtain player response for current current_round of Simon game. Returns current game Mode.
 */
uint8_t get_player_move()
{
	// set response time out period
	response_counter = RESPONSE_MS;
	
	// loop while waiting for player move		
	waiting = 1;
	do
	{
		// continuously cycle through the pseudorandom number
		// sequence while awaiting player input
		random = rand_lfsr( random );
		if ( !response_counter )
		{
			// timed out
			return LOSE;
		}	
		if ( buttons_pressed && !debounce_counter )
		{
			// inputs have changed and debounce time elapsed
			// get input from switches 
			get_input();
			// parse input
			current_state = parse_player_input();
			if ( current_state != REPEAT )
			{
				return current_state;
			}
		} 			
		// idle until next interrupt	
		sleep_mode();
	} 
	while ( waiting );
	// wait for all switches to be closed (except MODE_SWITCH)
	// NB this feature was also in the original game:
	// you could slow down your turn by holding down the switch without timing out
	do
	{
		// idle until next interrupt
		sleep_mode();
		// poll switches
		get_input();
	}
	while( buttons_pressed & 0x3F );
	clear_leds();
	return REPEAT;
}

/**
 * repeat_mode()
 *
 * \brief Player's turn for current current_round of Simon game. Returns current game Mode.
 */
uint8_t repeat_mode()
{
	uint8_t i; // current move number
	for ( i = 0; i < current_round; i++ )
	{
		// get move to match (2 bits)
		get_simon_move( i, &move );  
		
		// get player move
		current_state = get_player_move(); 
		if ( current_state != REPEAT )
		{
			return current_state;
		} 
	} // next move
	// completed current_round
	current_round++;
	disable_input();
	if ( current_round == MAX_NUM_MOVES )
	{
		return WIN;
	}
	if ( current_round == 9 )
	{
		// fanfare indicates start of new speed level
		delay_ms( DELAY_AFTER_REPEAT_MS );
		led_fanfare( 25, 50 );
	} else if ( current_round == 15 )
	{
		// fanfare indicates start of new speed level
		delay_ms( DELAY_AFTER_REPEAT_MS );
		led_fanfare( 49, 50 );
	} else if ( current_round == 21 )
	{
		// fanfare indicates start of new speed level
		delay_ms( DELAY_AFTER_REPEAT_MS );
		led_fanfare( 73, 50 );
	}	
	delay_ms( DELAY_AFTER_REPEAT_MS );
	// next current_round
	return LISTEN;
}

/**
 * win_mode()
 *
 * \brief Play win sequence. Returns current game Mode.
 */
uint8_t win_mode()
{
	make_noise( TUNE );
	led_fanfare( 150, 10 );
	delay_ms( DELAY_BETWEEN_GAMES_MS );
	return START_SIMON;
}

/**
 * lose_mode()
 *
 * \brief Play lose sequence. Returns current game Mode.
 */
uint8_t lose_mode()
{
	delay_ms( DELAY_AFTER_LOSS_MS );
	make_noise( CROW );
	// Strobing blinky lights say NO!!!
	uint8_t i;
	for ( i = 0; i < 10; i++ )
	{
		blink_leds();
		delay_ms( 25 );
	}
	delay_ms( DELAY_BETWEEN_GAMES_MS );
	return START_SIMON;
}

/**
 * main()
 *
 * \brief Entry program sequence containing main loop for Finite State Machine.
 */
int main( void )
{
	// initialize ports
	port_init();
	
	// initialize interrupts and other machine settings
	config();
	
	// set initial Mode
	// enter START_FARM mode unless MODE_SWITCH is engaged
	current_state = ( SWITCH_PORT & ( 1 << MODE_SWITCH ) ) ? START_SIMON : START_FARM;

	// main loop for Finite State Machine
	while ( 1 ) 
	{
		switch ( current_state )
		{
		case START_SIMON: 
			current_state = start_simon_mode();
			
		case LISTEN:
			current_state = listen_mode();
			
		case REPEAT:
			current_state = repeat_mode();
			break;
		
		case WIN:
			current_state = win_mode();
			break;
			
		case LOSE:
			current_state = lose_mode();
			break;
			
		case START_FARM:
			current_state = start_farm_mode();
			
		case FARM:
		default:
			current_state = farm_mode();
		}
		// idle until next interrupt
		sleep_mode();
	}
	return 0;
}

/******************************************************
 *                                                    *
 *       I N T E R R U P T    R O U T I N E S         *
 *                                                    *
 ******************************************************/

/**
 * Timer 0 countdown timer compare interrupt handler
 * wakes CPU every 1 ms and decrements timers
 */
ISR ( TIMER0_COMPA_vect ) 
{
	if ( delay_counter ) 
	{
		delay_counter--;
	}  
	if ( debounce_counter ) 
	{
		debounce_counter--;
	}  
	if ( response_counter ) 
	{
		response_counter--;
	}  
	if ( led_hold_counter ) 
	{
		led_hold_counter--;
	} else
	{
		clear_leds();
	}
	if ( noise_hold_counter ) 
	{
		noise_hold_counter--;
	} else
	{
		NOISE_PORT &= 0xC0;
		if ( next_noise != 0xFF )
		{
			NOISE_PORT |= ( 1 << next_noise );
			noise_hold_counter = next_noise_hold;
		}
		next_noise_hold = 0;
		next_noise = 0xFF;
	}
}

/**
 * Pin change interrupt handler
 * used in preference to polling switches to gain player input
 */
ISR( PCINT_vect )
{ 
	// turn off pin change interrupt during debounce
	GIMSK &= (uint8_t) ~( 1 << PCIE );
	// store input(s) from switches OR'd with flag bit
	buttons_pressed = SWITCH_PORT | 0x80;
	// set wait before next button press
	debounce_counter = DEBOUNCE_MS; 
} 
