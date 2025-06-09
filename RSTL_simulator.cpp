#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/select.h>
#include <inttypes.h>
#include <assert.h>

//.................................................................................................
// Preprocessor directives
//.................................................................................................

#define RSTL_HARDWARE_SPEED			B4800
#define TEXT_HARDWARE_SPEED			"4800"

#define MAX_COMMAND_LENGTH			100

//.................................................................................................
// Definitions of types
//.................................................................................................

//.................................................................................................
// Local constants
//.................................................................................................

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
	bool ExitFlag = false;
	bool WaitingForNewCommand = true;
	bool IsNewCommand = false;
	uint16_t TotalReceivedBytes;
	char TotalCommand[MAX_COMMAND_LENGTH];
	char OutgoingResponse[1000];

	// Serial port sutup
    for (int J = 1; J < argc; J++) {
        std::string Argument = argv[J];
       	std::cout << "argument " << J << " [" << Argument << "]" << std::endl;
    }
    if ( 3 != argc ){
    	std::cout << "Syntax error; example of a program call:  ./RSTL_simulator /dev/ttyS0 \"Rev 3.05 RSTL 7.5-300 Serial 97H-7004\"" << std::endl;
    	return 0;
    }
	SerialDevice = configureSerialPort( argv[1] );
	if (SerialDevice < 0){
		std::cout << "Port not opened" << std::endl;
		return 0;
	}

	// Keyboard reading setup
	struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO); // non-canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

    while (!ExitFlag) {

    	// Keyboard commands
        fd_set KeyboardReadFds;
        FD_ZERO(&KeyboardReadFds);
        FD_SET(STDIN_FILENO, &KeyboardReadFds);

        struct timeval KeyboardTimeout = {0, 0}; // wait 0 seconds
        int KeyboardState = select(STDIN_FILENO + 1, &KeyboardReadFds, nullptr, nullptr, &KeyboardTimeout);
        if (KeyboardState > 0 && FD_ISSET(STDIN_FILENO, &KeyboardReadFds)) {
            char KeyCode;
            if (read(STDIN_FILENO, &KeyCode, 1) == 1) {
        		if ((27 == KeyCode) || ('q' == KeyCode) || (1 == KeyCode)){  // Esc, q, Ctrl+A
                    break;
                }
            }
        }

        // Reading from the serial port and sending response
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
			if (NumberOfReceived > 0) {

				// Sending the echo
				int WrittenBytes = write(SerialDevice, &ReceivedBytes[0], NumberOfReceived );
				if (WrittenBytes != NumberOfReceived){
					std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
					ExitFlag = true;
					break;
				}

				for (int J=0; J < NumberOfReceived; J++){
					TotalCommand[TotalReceivedBytes] = ReceivedBytes[J];

					if (WaitingForNewCommand && ('\r' == ReceivedBytes[J])){
						WaitingForNewCommand = false;

						int OutgoingBytes = snprintf( &OutgoingResponse[0], sizeof(OutgoingResponse)-1, "It will be the answer here" );
						int WrittenBytes = write(SerialDevice, &OutgoingResponse[0], OutgoingBytes );
						if (WrittenBytes != OutgoingBytes){
							std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
							ExitFlag = true;
							break;
						}
						// Reset the state machine that receives commands and executes them
						TotalReceivedBytes = 0;
						WaitingForNewCommand = true;
						break; // exits the “for...” loop
					}

					if (TotalReceivedBytes > MAX_COMMAND_LENGTH-2){
						std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
						ExitFlag = true;
						break;
					}

					TotalReceivedBytes++;
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
			// There is nothing to do but wait
		}

        usleep(10);
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


