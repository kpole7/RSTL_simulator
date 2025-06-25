#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/select.h>
#include <inttypes.h>
#include <assert.h>
#include <string.h>

//.................................................................................................
// Preprocessor directives
//.................................................................................................

#define SERIAL_PORTS_ARE_VIRTUAL	1
#define VIRTUAL_PORT_DELAY			2500 // us

#define RSTL_HARDWARE_SPEED			B4800
#define TEXT_HARDWARE_SPEED			"4800"

#define MAX_COMMAND_LENGTH			100

//.................................................................................................
// Definitions of types
//.................................................................................................

//.................................................................................................
// Local constants
//.................................................................................................

static const char InquiryForSerialNumber[]       = "?M\r\n";
static const char InquiryForCurrentMeasurement[] = "MC\r\n";
static const char InquiryForOperationMode[]      = "?O\r\n";
static const char InquiryForCurrentDac[]         = "?C\r\n";
static const char CommandSetRemote[]             = "SR\r\n";
static const char CommandSetEchoOn[]             = "SB1\r\n";
static const char CommandSetShortOutput[]        = "SM0\r\n";

static const char EchoOnPrefix[]				 = "\n\r";

//.................................................................................................
// Local variables
//.................................................................................................

static int SerialDevice;
static char* RstlIdentifier;

static double SimulationCurrentValue = 1.0;
static double SimulationNoisyCurrentValue;
static double SimulationSetPointValue = 0.0;
static double SimulationTransientCoefficient1 = 0.01;
static double SimulationTransientCoefficient2 = 0.99;
static bool IsRemoteOperationMode(true);
static bool IsEchoOn(true);
static bool IsShortOutput(true);

//.................................................................................................
// Local function prototypes
//.................................................................................................

static int configureSerialPort(const char *DeviceName);

static int prepareResponse( char* OutputString, int OutputMaxLength, char* Command );

//........................................................................................................
// Main function definition
//........................................................................................................

int main(int argc, char** argv) {
	uint32_t Counter1;
	uint64_t Clocks1, Clocks2;
	bool ExitFlag = false;
	bool WaitingForNewCommand = true;
	bool IsNewCommand = false;
	uint16_t TotalReceivedBytes = 0;
	char TotalCommand[MAX_COMMAND_LENGTH];
	char OutgoingResponse[1000];
	uint16_t TimeDivider = 0;
	bool TransientPrintout = false;

	// Serial port sutup
    for (int J = 1; J < argc; J++) {
        std::string Argument = argv[J];
       	std::cout << "argument " << J << " [" << Argument << "]" << std::endl;
    }
    if ( 3 != argc ){
    	std::cout << "Syntax error; example of a program call:  ./RSTL_simulator /dev/ttyS0 \"Rev 3.05 RSTL 15-300 Serial 97J-7929\"" << std::endl;
//    	std::cout << "Syntax error; example of a program call:  ./RSTL_simulator /dev/ttyS0 \"Rev 3.05 RSTL 7.5-300 Serial 97H-7004\"" << std::endl;
    	return 0;
    }
	SerialDevice = configureSerialPort( argv[1] );
	if (SerialDevice < 0){
		std::cout << "Port not opened" << std::endl;
		return 0;
	}
	RstlIdentifier = argv[2];

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
            	if ('f' == KeyCode){
            		SimulationTransientCoefficient1 = 0.05;
            		SimulationTransientCoefficient2 = 0.95;
            		std::cout << "Transient: fast" << std::endl;
            	}
            	if ('d' == KeyCode){
            		SimulationTransientCoefficient1 = 0.01;
            		SimulationTransientCoefficient2 = 0.99;
            		std::cout << "Transient: default" << std::endl;
            	}
            	if ('s' == KeyCode){
            		SimulationTransientCoefficient1 = 0.002;
            		SimulationTransientCoefficient2 = 0.998;
            		std::cout << "Transient: slow" << std::endl;
            	}
            	if ('p' == KeyCode){
            		if (TransientPrintout){
            			TransientPrintout = false;
            			std::cout << "Transient printouts: off" << std::endl;
            		}
            		else{
            			TransientPrintout = true;
            			std::cout << "Transient printouts: on" << std::endl;
            		}
            	}
            	if ('v' == KeyCode){
            		if (IsShortOutput){
            			IsShortOutput = false;
            			std::cout << "verbose output" << std::endl;
            		}
            		else{
            			IsShortOutput = true;
            			std::cout << "short output" << std::endl;
            		}
            	}
            	if ('l' == KeyCode){
            		if (IsRemoteOperationMode){
            			IsRemoteOperationMode = false;
            			std::cout << "local mode" << std::endl;
            		}
            		else{
            			IsRemoteOperationMode = true;
            			std::cout << "remote mode" << std::endl;
            		}
            	}
            	if ('e' == KeyCode){
            		if (IsEchoOn){
            			IsEchoOn = false;
            			std::cout << "echo off" << std::endl;
            		}
            		else{
            			IsEchoOn = true;
            			std::cout << "echo on" << std::endl;
            		}
            	}
            	if ('r' == KeyCode){
            		if (drand48() > 0.5){
            			IsEchoOn = true;
               			std::cout << "echo on; ";
            		}
            		else{
            			IsEchoOn = false;
               			std::cout << "echo off; ";
            		}
            		if (drand48() > 0.5){
            			IsRemoteOperationMode = true;
               			std::cout << "remote mode; ";
            		}
            		else{
            			IsRemoteOperationMode = false;
               			std::cout << "local mode; ";
            		}
            		if (drand48() > 0.5){
            			IsShortOutput = true;
               			std::cout << "short output" << std::endl;
            		}
            		else{
            			IsShortOutput = false;
               			std::cout << "verbose output" << std::endl;
            		}
            	}
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
#if 0 == SERIAL_PORTS_ARE_VIRTUAL
				int WrittenBytes = write(SerialDevice, &ReceivedBytes[0], NumberOfReceived );
				if (WrittenBytes != NumberOfReceived){
					std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
					ExitFlag = true;
					break;
				}
#else
				for (int J=0; J<NumberOfReceived; J++ ){
					int WrittenBytes = write(SerialDevice, &ReceivedBytes[J], 1 );
					if (WrittenBytes != 1){
						std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
						ExitFlag = true;
						break;
					}
					 usleep( VIRTUAL_PORT_DELAY );
				}
#endif
				// reading byte by byte
				for (int J=0; J < NumberOfReceived; J++){
					TotalCommand[TotalReceivedBytes] = ReceivedBytes[J];
					// check if there is a complete command
					if (WaitingForNewCommand && ('\n' == ReceivedBytes[J])){
						WaitingForNewCommand = false;
						TotalCommand[TotalReceivedBytes+1] = 0;
#if 1
						std::cout << "->  ";
						for (int J=0; J < TotalReceivedBytes; J++){
							if (10 == TotalCommand[J]){
								std::cout << "\\n";
							}
							else if (13 == TotalCommand[J]){
								std::cout << "\\r";
							}
							else if ((' ' <= TotalCommand[J]) || ('z' >= TotalCommand[J])){
								std::cout << TotalCommand[J];
							}
							else{
								std::cout << (char)128;
							}
						}
						std::cout << std::endl;
#endif

						// command response
						if (IsEchoOn || (0 == strncmp( TotalCommand, CommandSetEchoOn, sizeof(CommandSetEchoOn)-1))){
#if 0 == SERIAL_PORTS_ARE_VIRTUAL
							int WrittenBytes = write(SerialDevice, &EchoOnPrefix[0], 2 );
							if (WrittenBytes != 2){
								std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
								ExitFlag = true;
								break;
							}
#else
							for (int J=0; J<2; J++ ){
								int WrittenBytes = write(SerialDevice, &EchoOnPrefix[J], 1 );
								if (WrittenBytes != 1){
									std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
									ExitFlag = true;
									break;
								}
								 usleep( VIRTUAL_PORT_DELAY );
							}
#endif
						}
						if (0 == strncmp( TotalCommand, InquiryForCurrentMeasurement, sizeof(InquiryForCurrentMeasurement)-1)){
					        usleep(600000LU);
						}

						int OutgoingBytes = prepareResponse( &OutgoingResponse[0], sizeof(OutgoingResponse)-1, TotalCommand );
						if (OutgoingBytes < 0){
							std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
							ExitFlag = true;
							break;
						}

#if 0 == SERIAL_PORTS_ARE_VIRTUAL
						int WrittenBytes = write(SerialDevice, &OutgoingResponse[0], OutgoingBytes );
						if (WrittenBytes != OutgoingBytes){
							std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
							ExitFlag = true;
							break;
						}
#else
						for (int J=0; J<OutgoingBytes; J++ ){
							int WrittenBytes = write(SerialDevice, &OutgoingResponse[J], 1 );
							if (WrittenBytes != 1){
								std::cout << "Exiting " << __FILE__ << ": " << __LINE__ << std::endl;
								ExitFlag = true;
								break;
							}
							 usleep( VIRTUAL_PORT_DELAY );
						}
#endif

#if 1 // debugging
						std::cout << " <- ";
						for (int J=0; J < OutgoingBytes; J++){
							if (10 == OutgoingResponse[J]){
								std::cout << "\\n";
							}
							else if (13 == OutgoingResponse[J]){
								std::cout << "\\r";
							}
							else if ((' ' <= OutgoingResponse[J]) || ('z' >= OutgoingResponse[J])){
								std::cout << OutgoingResponse[J];
							}
							else{
								std::cout << (char)128;
							}
						}
						std::cout << std::endl;
#endif
						// Reset the state machine that receives commands and executes them
						TotalReceivedBytes = 0;
						WaitingForNewCommand = true;
						break; // exits the "for..." loop
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

		if (0 == TimeDivider){
			TimeDivider = 256;

			SimulationCurrentValue = SimulationTransientCoefficient2 * SimulationCurrentValue + SimulationTransientCoefficient1 * SimulationSetPointValue;

			double SimulationRandomFloat = drand48() - 0.5;
			for(int J=0;J<9;J++){
				SimulationRandomFloat += drand48() - 0.5;
			}
			SimulationRandomFloat *= SimulationCurrentValue/50.0; // The value of the constant was picked experimentally
			SimulationNoisyCurrentValue = SimulationCurrentValue + SimulationRandomFloat;
			if (SimulationNoisyCurrentValue < 0){
				SimulationNoisyCurrentValue = 0;
			}
			if (TransientPrintout){
				std::cout << SimulationCurrentValue << std::endl;
			}
		}
		else{
#if 0
			if (100 == TimeDivider){
				Clocks1 = clock();
				time_t now = time(NULL);
				std::cout << Counter1 << "    clock= " << Clocks1 << "  dif= " << " (" << (uint64_t)(Clocks1/CLOCKS_PER_SEC) << ") " << Clocks1-Clocks2 << "  time= " << (uint32_t)now << " " << ctime(&now);
			}
			if (101 == TimeDivider){
				Clocks2 = clock();
				time_t now = time(NULL);
				std::cout << Counter1 << "    clock= " << Clocks2 << "  dif= " << " (" << (uint64_t)(Clocks2/CLOCKS_PER_SEC) << ") "<< Clocks2-Clocks1 << "  time= " << (uint32_t)now << " " << ctime(&now);
			}
#endif
			TimeDivider--;
		}

		Counter1++;
        usleep(1000);
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

static int prepareResponse( char* OutputString, int OutputMaxLength, char* Command ){
	int Result = 0;
	static char InternalCharacter;

	if (0 == strncmp( Command, InquiryForSerialNumber, sizeof(InquiryForSerialNumber)-1)){
		if (IsShortOutput){
			Result = snprintf( &OutputString[0], OutputMaxLength, "%s\r\n\n>", RstlIdentifier );
		}
		else{
			Result = snprintf( &OutputString[0], OutputMaxLength, "%s\r\n\nCommand>", RstlIdentifier );
		}
		if (Result > OutputMaxLength){
			std::cerr << "Error " << __FILE__ << " " << __LINE__ << std::endl;
			Result = -1;
		}
	}
	else if (0 == strncmp( Command, InquiryForCurrentMeasurement, sizeof(InquiryForCurrentMeasurement)-1)){
		if (IsShortOutput){
			Result = snprintf( &OutputString[0], OutputMaxLength, "%.2f\r\n\n>", SimulationNoisyCurrentValue );
		}
		else{
			Result = snprintf( &OutputString[0], OutputMaxLength, "Current = %.2f Amps \r\n\nCommand>", SimulationNoisyCurrentValue );
		}
	}
	else if (0 == strncmp( Command, InquiryForOperationMode, sizeof(InquiryForOperationMode)-1)){
		if (IsRemoteOperationMode){
			InternalCharacter = 'R';
		}
		else{
			InternalCharacter = 'L';
		}
		if (IsShortOutput){
			Result = snprintf( &OutputString[0], OutputMaxLength, "%c\r\n\n>", InternalCharacter );
		}
		else{
			Result = snprintf( &OutputString[0], OutputMaxLength, "%c operation\r\n\nCommand>", InternalCharacter );
		}
	}
	else if (0 == strncmp( Command, InquiryForCurrentDac, sizeof(InquiryForCurrentDac)-1)){
		if (IsShortOutput){
			Result = snprintf( &OutputString[0], OutputMaxLength, "PCurrent = %.3f Amps\r\n\n>", SimulationSetPointValue );
		}
		else{
			Result = snprintf( &OutputString[0], OutputMaxLength, "PCurrent = %.3f Amps\r\n\nCommand>", SimulationSetPointValue );
		}
	}
	else if (0 == strncmp( Command, CommandSetRemote, sizeof(CommandSetRemote)-1)){
		IsRemoteOperationMode = true;
		if (IsShortOutput){
			Result = snprintf( &OutputString[0], OutputMaxLength, ">\r\n\n>" );
		}
		else{
			Result = snprintf( &OutputString[0], OutputMaxLength, ">\r\n\nCommand>" );
		}
	}
	else if (0 == strncmp( Command, CommandSetEchoOn, sizeof(CommandSetEchoOn)-1)){
		IsEchoOn = true;
		if (IsShortOutput){
			Result = snprintf( &OutputString[0], OutputMaxLength, ">\n\r\r\n\n>" );
		}
		else{
			Result = snprintf( &OutputString[0], OutputMaxLength, ">\n\r\r\n\nCommand>" );
		}
	}
	else if (0 == strncmp( Command, CommandSetShortOutput, sizeof(CommandSetShortOutput)-1)){
		IsShortOutput = true;
		Result = snprintf( &OutputString[0], OutputMaxLength, "\r\r\n\n>" );
	}
	else{
		int CommandLength = strlen(Command);
		if (CommandLength < MAX_COMMAND_LENGTH-1){
#if 0
			for (int J=0; 0 != Command[J]; J++){
				if ((' ' < Command[J]) && ('z' >= Command[J])){
					std::cout << " " << Command[J];
				}
				else{
					std::cout << " 0x" << (int)Command[J];
				}
			}
			std::cout << "}" << std::dec << std::endl;
#endif
			if (('P' == Command[0]) && ('C' == Command[1]) && ('\r' == Command[CommandLength-2]) && ('\n' == Command[CommandLength-1])){
				char* EndPtr;
				double CommandSetPoint = strtof( &Command[2], &EndPtr );
#if 1
				std::cout << "CommandSetPoint " << CommandSetPoint << " " << (int)(EndPtr-Command) << " " << CommandLength << std::endl;
#endif
				if ((int)(EndPtr-Command)+2 == CommandLength){ // Check if the number format is correct
					if (IsShortOutput){
						Result = snprintf( &OutputString[0], OutputMaxLength, "\r\n\n>" );
					}
					else{
						Result = snprintf( &OutputString[0], OutputMaxLength, "\r\n\nCommand>" );
					}
					if (Result > OutputMaxLength){
						Result = -1;
					}
					SimulationSetPointValue = CommandSetPoint;
				}
			}
		}
	}
	return Result;
}

