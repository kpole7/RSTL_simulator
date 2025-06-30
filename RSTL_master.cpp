#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>
#include <chrono>
#include <sys/select.h>
#include <inttypes.h>
#include <assert.h>


//.................................................................................................
// Preprocessor directives
//.................................................................................................

#define RSTL_HARDWARE_SPEED			B4800
#define TEXT_HARDWARE_SPEED			"4800"

#define FRAMES_NUMBER				24

#define MAX_RESPONSE_LENGTH			100

//.................................................................................................
// Definitions of types
//.................................................................................................

typedef struct FrameInfoStruct{
	const uint8_t *outgoingFramePtr;
	const uint8_t outgoingFrameLength;
}FrameInfo;

//.................................................................................................
// Local constants
//.................................................................................................

static const uint8_t FrameToBeSent01[] = "PC0\r\n";		// 1 Program Current to...
static const uint8_t FrameToBeSent02[] = "PC0.2\r\n";	// 2 Program Current to...
static const uint8_t FrameToBeSent03[] = "PC1\r\n";		// 3 Program Current to...
static const uint8_t FrameToBeSent04[] = "PC2.1\r\n";	// 4 Program Current to...
static const uint8_t FrameToBeSent05[] = "PC5.1\r\n";	// 5 Program Current to...

static const uint8_t FrameToBeSent06[] = "PCX10\r\n";	// 8 DAC 0x010
static const uint8_t FrameToBeSent07[] = "PCX20\r\n";	// 9 DAC 0x020

static const uint8_t FrameToBeSent08[] = "?M\r\n";		// A Place Software Revision, Model and Serial number
static const uint8_t FrameToBeSent09[] = "MC\r\n";		// B measure output current and return result in Amps format
static const uint8_t FrameToBeSent10[] = "MCX\r\n";		// C measure output current and return result in hex format
static const uint8_t FrameToBeSent11[] = "?C\r\n";		// D Current DAC programming value (decimal)
static const uint8_t FrameToBeSent12[] = "?CX\r\n";		// E Current DAC programming value (hex)
static const uint8_t FrameToBeSent13[] = "?O\r\n";		// F Local or remote operation

static const uint8_t FrameToBeSent14[] = "?VL\r\n";		// G limit
static const uint8_t FrameToBeSent15[] = "?VLX\r\n";	// H
static const uint8_t FrameToBeSent16[] = "?CL\r\n";		// I
static const uint8_t FrameToBeSent17[] = "?CLX\r\n";	// J

static const uint8_t FrameToBeSent18[] = "?VL\r\n";		// K
static const uint8_t FrameToBeSent19[] = "?CL\r\n";		// L

static const uint8_t FrameToBeSent20[] = "?TV\r\n";		// M trimming
static const uint8_t FrameToBeSent21[] = "?TC\r\n";		// N

static const uint8_t FrameToBeSent22[] = "SR\r\n";		// ! Remote operation
static const uint8_t FrameToBeSent23[] = "SB1\r\n";		// @ Echo on
static const uint8_t FrameToBeSent24[] = "SM0\r\n";		// # Short output

static const FrameInfo FrameInfoTable[FRAMES_NUMBER] = {
		{	FrameToBeSent01,	sizeof(FrameToBeSent01)-1	},
		{	FrameToBeSent02,	sizeof(FrameToBeSent02)-1	},
		{	FrameToBeSent03,	sizeof(FrameToBeSent03)-1	},
		{	FrameToBeSent04,	sizeof(FrameToBeSent04)-1	},
		{	FrameToBeSent05,	sizeof(FrameToBeSent05)-1	},
		{	FrameToBeSent06,	sizeof(FrameToBeSent06)-1	},
		{	FrameToBeSent07,	sizeof(FrameToBeSent07)-1	},
		{	FrameToBeSent08,	sizeof(FrameToBeSent08)-1	},
		{	FrameToBeSent09,	sizeof(FrameToBeSent09)-1	},
		{	FrameToBeSent10,	sizeof(FrameToBeSent10)-1	},
		{	FrameToBeSent11,	sizeof(FrameToBeSent11)-1	},
		{	FrameToBeSent12,	sizeof(FrameToBeSent12)-1	},
		{	FrameToBeSent13,	sizeof(FrameToBeSent13)-1	},
		{	FrameToBeSent14,	sizeof(FrameToBeSent14)-1	},
		{	FrameToBeSent15,	sizeof(FrameToBeSent15)-1	},
		{	FrameToBeSent16,	sizeof(FrameToBeSent16)-1	},
		{	FrameToBeSent17,	sizeof(FrameToBeSent17)-1	},
		{	FrameToBeSent18,	sizeof(FrameToBeSent18)-1	},
		{	FrameToBeSent19,	sizeof(FrameToBeSent19)-1	},
		{	FrameToBeSent20,	sizeof(FrameToBeSent20)-1	},
		{	FrameToBeSent21,	sizeof(FrameToBeSent21)-1	},
		{	FrameToBeSent22,	sizeof(FrameToBeSent22)-1	},
		{	FrameToBeSent23,	sizeof(FrameToBeSent23)-1	},
		{	FrameToBeSent24,	sizeof(FrameToBeSent24)-1	},
};

static_assert( FRAMES_NUMBER == (int)(sizeof(FrameInfoTable)/sizeof(FrameInfoTable[0])));

//                                 123456789012345678901234567890
static char KeybordCharacters[] = "1234589ABCDEFGHIJKLMN!@#";

//.................................................................................................
// Local variables
//.................................................................................................

static int SerialDevice;

//.................................................................................................
// Local function prototypes
//.................................................................................................

static int configureSerialPort(const char *DeviceName);

//........................................................................................................
// Main function definition
//........................................................................................................

int main(int argc, char** argv) {
	uint16_t TotalReceivedBytes;
	char TotalResponse[MAX_RESPONSE_LENGTH];
	bool NewLineCharacter = false;

	// Serial port setup
    for (int J = 1; J < argc; J++) {
        std::string Argument = argv[J];
       	std::cout << "argument " << J << " [" << Argument << "]" << std::endl;
    }
    if ( 2 != argc ){
    	std::cout << "Syntax error; example of a program call:  ./RSTL_master /dev/pts/5" << std::endl;
    	return 0;
    }
	SerialDevice = configureSerialPort( argv[1] );
	if (SerialDevice < 0){
		std::cout << "Port not opened" << std::endl;
		return 0;
	}
    std::cout << "Port open and configured; handler=" << SerialDevice
    		<< "  baudrate=" << TEXT_HARDWARE_SPEED << std::endl;

    struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO); // non-canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

	auto TimeStart = std::chrono::high_resolution_clock::now();
    auto TimeNow = TimeStart;
    auto TimeBefore = TimeStart;

    std::cout << "size of variable defined with:   std::chrono::high_resolution_clock::now()  = " << (int)sizeof(TimeStart) << std::endl;

    uint32_t TestCounter = 0;

    while (true) {
        fd_set KeyboardReadFds;
        FD_ZERO(&KeyboardReadFds);
        FD_SET(STDIN_FILENO, &KeyboardReadFds);

        struct timeval KeyboardTimeout = {0, 0}; // wait 0 seconds
        int KeyboardState = select(STDIN_FILENO + 1, &KeyboardReadFds, nullptr, nullptr, &KeyboardTimeout);
        if (KeyboardState > 0 && FD_ISSET(STDIN_FILENO, &KeyboardReadFds)) {
            char KeyCode;
            if (read(STDIN_FILENO, &KeyCode, 1) == 1) {
                std::cout << "Pressed: " << (int)KeyCode;
                if (0 != isprint(KeyCode)){
                	std::cout << " " << KeyCode;
                }
                std::cout << "\n";
        		if ((27 == KeyCode) || ('q' == KeyCode) || (1 == KeyCode)){  // Esc, q, Ctrl+A
                    break;
                }
        		if ((' ' <= KeyCode) && (KeyCode <= 'z')){
        			char* FoundCharacterPtr = strchr( KeybordCharacters, KeyCode );

        			if (FoundCharacterPtr != nullptr){
            			int Index = FoundCharacterPtr - KeybordCharacters;
            			assert( Index < FRAMES_NUMBER );

            			TimeBefore = std::chrono::high_resolution_clock::now();
            			int n = write(SerialDevice, FrameInfoTable[Index].outgoingFramePtr, FrameInfoTable[Index].outgoingFrameLength );
        				TimeStart = std::chrono::high_resolution_clock::now();
        				TotalReceivedBytes = 0;
            			if (FrameInfoTable[Index].outgoingFrameLength == n){
            				std::cout << "Frame sent successfully";
            			}
            			else{
                        	std::cout << "Error sending frame";
            			}
        				std::cout << " (";
        				for (int J=0; J < FrameInfoTable[Index].outgoingFrameLength; J++){
        					std::cout << (char)(((FrameInfoTable[Index].outgoingFramePtr[J] >= ' ') && (FrameInfoTable[Index].outgoingFramePtr[J] <= 'z'))?
        							FrameInfoTable[Index].outgoingFramePtr[J] : '.');
        				}
        				std::cout << ")" << std::endl;
        				TestCounter = 0;
        			}
        		}
            }
        }

        fd_set SerialReadFds;
		FD_ZERO(&SerialReadFds);
		FD_SET(SerialDevice, &SerialReadFds);
		struct timeval SerialTimeout = { 0, 0 };

		int SerialPortState = select(SerialDevice + 1, &SerialReadFds, nullptr, nullptr, &SerialTimeout);

		if (SerialPortState == -1) {
			std::cerr << "Error: select()" << std::endl;
			break;
		}
		else if (SerialPortState > 0 && FD_ISSET(SerialDevice, &SerialReadFds)) {
			char ReceivedBytes[1000];
			int NumberOfReceived = read(SerialDevice, &ReceivedBytes[0], sizeof(ReceivedBytes));
			assert( NumberOfReceived <= sizeof(ReceivedBytes));

			if ((NumberOfReceived > 0) && (TotalReceivedBytes + (uint16_t)NumberOfReceived < MAX_RESPONSE_LENGTH-2)) {
				for (int J=0; J < NumberOfReceived; J++){
					TotalResponse[TotalReceivedBytes] = ReceivedBytes[J];
					if (TotalReceivedBytes > 0){
						if (('\n' == TotalResponse[TotalReceivedBytes-1]) && ('>' == TotalResponse[TotalReceivedBytes])){
							NewLineCharacter = true;
						}
					}
					TotalReceivedBytes++;
				}
				TotalResponse[TotalReceivedBytes] = 0;

				if (NewLineCharacter){
					NewLineCharacter = false;
					TimeNow = std::chrono::high_resolution_clock::now();
					auto Duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeNow - TimeStart);
					std::cout << "Time " << std::dec << Duration.count() << "ms ";
#if 0
					std::cout << "\tRec " << NumberOfReceived << " [" << TotalReceivedBytes << "]  ";
					for (int J=0; J < NumberOfReceived; J++){
						std::cout << std::hex << ((unsigned)ReceivedBytes[J]) % 256u << " ";
					}
#endif
					std::cout << "\t";
					for (int J=0; J < TotalReceivedBytes; J++){
						if (10 == TotalResponse[J]){
							std::cout << "\\n";
						}
						else if (13 == TotalResponse[J]){
							std::cout << "\\r";
						}
						else if ((' ' <= TotalResponse[J]) || ('z' >= TotalResponse[J])){
							std::cout << TotalResponse[J];
						}
						else{
							std::cout << (char)128;
						}
					}

					Duration = std::chrono::duration_cast<std::chrono::milliseconds>(TimeStart - TimeBefore);
					std::cout << "  T=" << std::dec << Duration.count() << "ms";

					std::cout << std::endl;
				}
			}
			else if (NumberOfReceived == -1) {
				std::cerr << "Error: read()" << std::endl;
			}
			else{
				std::cerr << "Error: received bytes " << NumberOfReceived << std::endl;
			}
		}
		else{
			// There is nothing to do
		}

        usleep(1000);
        TestCounter++;
    }

	close( SerialDevice );
    std::cout << "Port closed" << std::endl;

    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    return 0;
}

//........................................................................................................
// Local function definitions
//........................................................................................................

// This function opens and configures a serial port
static int configureSerialPort(const char *DeviceName){
    int FileHandler;
    struct termios PortSettings;

    FileHandler = open(DeviceName, O_RDWR | O_NOCTTY | O_SYNC  | O_NONBLOCK);
    if (FileHandler == -1) {
        return -1;
    }

    if (tcgetattr(FileHandler, &PortSettings) != 0) {
        close(FileHandler);
        return -1;
    }

    cfsetispeed(&PortSettings, RSTL_HARDWARE_SPEED);
    cfsetospeed(&PortSettings, RSTL_HARDWARE_SPEED);

    PortSettings.c_oflag = 0;                            // Disable output processing
    PortSettings.c_lflag = 0;                            // Mode 'raw'

    // no flow control
    PortSettings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

    PortSettings.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD | CRTSCTS);
    PortSettings.c_cflag |= (CS8 | CREAD | CLOCAL);

    PortSettings.c_cc[VMIN]  = 0;                        // Minimum number of bytes to read
    PortSettings.c_cc[VTIME] = 0;                        // Timeout in tenth of a second

    // apply the configuration
    if (tcsetattr(FileHandler, TCSANOW, &PortSettings) != 0) {
        close(FileHandler);
        return -1;
    }

    return FileHandler;
}


