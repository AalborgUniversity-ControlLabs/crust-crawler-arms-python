import serial
import random
import time
import array

class dynamixel_mx(object):
    """Communication with Dynamixel MX servos."""
    def __init__(self, port, baudrate=57600, timeout=0.02, DtrForRS485=False):
        """Entry point for communicating with the Dynamixel servos.

        Args:
            port (string/int): The port that the UartSBee is on.
                It may be a string '/dev/ttyUSB0' (Linux), 'COM9' (Windows)
                or a number. On Windows numbers seem to work best. In that
                case COM9 is number 8.
            baudrate (optional[int]): This is the baud rate that the servos
                communicate at. Defaults to 57600, which is the factory
                default of the Dynamixels, but you are likely to work
                with servos configured to 1000000 baud.
            timeout (optional[bool]): When issuing commands, the port will
                block execution while waiting for response. If you would
                rather like to discard "late" responses due to ex. real time
                constraints, set the timeout to match your constraints.
            DtrForRS485 (optional[bool]): If you use a USB serial port,
                e.g. a UartSBee to interface to a RS485 level changer, the
                DTR can be used to set the driver to send or receive.
                Default is False, so DTR will not be used.

        """

        # Initialize the port
        # A timeout of 5 ms will give the servos
        # ample time to respond
        self.serial_port = serial.Serial(port,
                    baudrate=baudrate,
                    timeout=timeout)

        self._DtrForRS485 = DtrForRS485

    def _construct_command(self, id, instruction, parameters=None):
        # Preample 0xFF 0xFF
        command = [0xFF, 0xFF]
        # ID
        command.append(id)
        # Length
        if isinstance(parameters, list):
            l = len(parameters)
        elif parameters is None:
            l = 0
        else:
            l = 1
        command.append(2 + l)
        # Instruction (write data = 0x03)
        command.append(instruction)
        # Parameters
        if parameters:
            command.extend(parameters)
        # Checksum
        checksum = (~(sum(command[2:]) & 0xFF)) & 0xFF
        command.append(checksum)
        return command


    def _send_command(self, command, response_length, verbose=False):
        if verbose:
            print("Sending:")
            print(command)
        buf = bytearray(command)
        # We use the DTR as enable/disable for
        # the RS485 driver. (This is because it
        # it is broken out on the UartSBee, that
        # we use.) True means receive and False
        # transmit.
        if self._DtrForRS485:
            self.serial_port.setDTR(False)
        bytes_written = self.serial_port.write(buf)
        if self._DtrForRS485:
            time.sleep(float(len(command))/self.serial_port.baudrate*10)
            self.serial_port.setDTR(True)
        if verbose:
            print("%i bytes written" % bytes_written)

        # Wait for the status package
        #time.sleep(0.1)

        # Read response
        # The status packet will not have any
        # payload so it will be 6 bytes long.
        response = self.serial_port.read(6+response_length)
        resp_list = array.array('B', response).tolist()
        if verbose:
            print("Status received:")
            print(resp_list)
            if self.check_checksum(resp_list):
                print("Checksum checks out.")

        return resp_list

    def ping(self, id, verbose=False):
        """ Ping a servo.

        Args:
            id (int): The ID of the servo may be in the range [1, 253].
                254 is the broadcast id, which the servos will not
                respond to, as they would all be responding at once.
            verbose (optional[bool]) Defaults to False, set to True
                to get more information on what is going on.

        Returns:
            True if the servo responded, False otherwise.

        """
        command = self._construct_command(id, 0x01)
        # Clear the input buffer to prepare for the response
        self.serial_port.reset_input_buffer()
        # Write the command to the bus
        response = self._send_command(command, 0, verbose=verbose)
        
        return self.check_checksum(response)

    def write_data(self, id, start_address, data, verbose=False):
        """ Write data to the registers of the servos.

        Args:
            id (int): ID of the servo.
            start_address (int): The first address that we will write to.
            data (list[int]/int): The data that should be written.
                If several bytes are to be written, data should be a list,
                otherwise it may be a single integer. Note that the values
                in data should not exceed 255.
            verbose (optional[bool]) print debug information, defaults to
                False.

        Returns:
            The status packet from the servo. If no packet was
            received, False is returned.

        """
        # Parameters (start address and value(s))
        parameters = [start_address]
        if isinstance(data, list):
            parameters.extend(data)
        else:
            parameters.append(data)
        command = self._construct_command(id, 0x03, parameters)
        # Clear the input buffer to prepare for the response
        self.serial_port.reset_input_buffer()
        # Write the command to the bus
        response = self._send_command(command, 0, verbose=verbose)

        if self.check_checksum(response):
            return response
        else:
            return False

    def read_data(self, id, start_address, length=1, verbose=False):
        """Read data from the internal registers of a servo.

        Args:
            id (int): The servo to query.
            start_address (int): The address to read the first byte from.
            length (optional[int]): How many bytes to read. Defaults to 1.
            verbose (optional): Get debug info. Defaults to False.

        Returns:
            The requested bytes in a list.

        """
        command = self._construct_command(id, 0x02, [start_address, length])
        # Clear the input buffer to prepare for the response
        self.serial_port.reset_input_buffer()
        # Write the command to the bus
        response = self._send_command(command, length, verbose=verbose)

        if self.check_checksum(response):
            return response[5:-1]
        else:
            return False

    def check_checksum(self, packet):
        if not packet:
            return False
        my_checksum = (~(sum(packet[2:-1]) & 0xFF)) & 0xFF
        if my_checksum == packet[-1]:
            return True
        else:
            return False

    def torque_enable(self, id, enable):
        """Enable consant torque on a servo.

        Args:
            id (int/list[int]): id(s) to en-/disable
            enable (bool): True to enable, False to disable.

        Returns:
            Status packet, False if none was received.
        """

        if not(1 <= id <= 254):
            raise ValueError("Target ID not in range [1, 254]")

        enable_data = 0
        if enable:
            enable_data = 1
        if isinstance(id, list):
            for i in id:
                response = self.write_data(id=i, start_address=24, data=enable_data)
        else:
            response = self.write_data(id=id, start_address=24, data=enable_data)
        return response

    def set_position(self, id, position):
        """Set the position of a servo.

        Convenience function for setting the position setpoint. Underneath
        it uses the write_data method.

        Args:
            id (int): Servo to move.
            position(int): Position in the range defined by the servo. The MX
                servos have a resolution of 0xFFF for 360 degrees, but they
                might have soft limits restraining the movement.

        Returns:
            The status packet returned by the servo. False if no packet
            was received.

        """
        setpoint_data = []
        setpoint_data.append(position & 0xFF)
        setpoint_data.append((position & 0xFF00) >> 8)
        response = self.write_data(id=id, start_address=30, data=setpoint_data)
        return response

    def get_position(self, id):
        """Query a servo for its position."""
        p = self.read_data(id, start_address=36, length=2)
        pos = p[0] + p[1]*256
        return pos

    def get_voltage(self, id):
        """Get the voltage in the servo."""
        v = self.read_data(id, start_address=42, length=1)
        print(v)
        return v[0]/10.0

    def set_baudrate(self, rate):
        """Set a new baud rate.

        The new baudrate is broadcast, so that an entire connected robot
        may be reset with one call.

        The PC serial port is also set to the new rate.

        Args:
            rate (int): The new desired rate.

        """
        baud_code = int((2000000/rate)-1)
        response = self.write_data(id=254, start_address=4, data=baud_code)
        time.sleep(0.01)
        self.serial_port.baudrate = rate
        return response

    def set_id(self, id, new_id, verbose=False):
        """Set a new id for a servo.

        CAUTION: If you have more than one servo with the same ID, they will
        both change their ID to the same new ID.

        Args:
            id(int): Old ID
            new_id(int): New ID, the ID must be in the range [1, 253].

        Returns:
            The status packet received from the servo. False, if nothing
            was received.

        """
        if not(1 <= id <= 253):
            raise ValueError("Target ID not in range [1, 253]")

        if not(1 <= new_id <= 253):
            raise ValueError("New ID not in range [1, 253]")

        response = self.write_data(id=id, start_address=3, data=new_id, verbose=verbose)
        return response

    def set_pid(self, id, k_p, k_i, k_d):
        """Set the PID gains of an MX servo

        id: The ID of the servo to setup.
        k_p: The proportional gain [0; 31.75] (default for dynamixels is 4)
        k_i: The integral gain [0; 124.02] (default for dynamixels is 0)
        k_d: The diferential gain [0; 1.02] (default for dynamixels is 0)
        """
        p_gain = int(k_p * 8)
        self.write_data(id=id, start_address=28, data=p_gain)

        i_gain = int(k_i * 2.048)
        self.write_data(id=id, start_address=27, data=i_gain)

        d_gain = int(k_d * 250)
        self.write_data(id=id, start_address=26, data=d_gain)

# Here comes some recovery tools:

    def scan_ids(self, verbose=False):
        """Scan for IDs on the bus.

        Returns:
            A list of found IDs.

        """
        # Save the current timeout. If the time out is None
        # we will never return from pinging a non existing servo.
        current_timeout = self.serial_port.timeout
        if current_timeout is not None and current_timeout < 0.02:
            self.serial_port.timeout = 0.02

        hits = []
        for i in range(1,254):
            if self.ping(i, verbose=verbose):
                hits.append(i)
        
        # Write the timeout back
        self.serial_port.timeout = current_timeout

        return hits

    def scan_baud(self, bauds, id=1):
        """Scan a range of baudrates for a servo.

        You know the ID of a servo but not the baudrate. This is the
        method to use.

        Args:
            bauds (list[int]): A list of the rates you want to examine.
            id (optional[int]): The ID to search for. Defaults to 1.

        Returns:
            The baudrate, if a servo was found, otherwise False.

        Example:
            Search the standard rates for ID 1:
                rates = [9600, 19200, 57600, 115200, 200000, 250000, 400000, 500000, 1000000]
                resp = servos.scan_baud(rates)
                print(resp)

        """
        # Test for valid ID
        if not(1 <= id <= 254):
            raise ValueError("ID not in the range [1, 254]")
        if id == 254:
            raise ValueError("Cannot scan on broadcast ID (254)")

        hit = False

        # Save the current timeout. If the time out is None
        # we will never return from pinging a non existing servo.
        current_timeout = self.serial_port.timeout
        self.serial_port.timeout = 0.01
        # Same for the baud rate
        current_baud = self.serial_port.baudrate
        for b in bauds:
            self.serial_port.baudrate = b
            time.sleep(0.0010)
            if self.ping(id):
                hit = b
                break

        # Write the timeout and baud rate back
        self.serial_port.timeout = current_timeout
        self.serial_port.baudrate = current_baud
        return hit

    def brute_force_scan(self, bauds):
        """ Find a servo with an unknown ID on and unknown baudrate.

        OK, you've set the baud rate to something stupid, or you just found
        a servo in the corner of the lab. This method will search for all
        possible IDs on a range of frequencies.

        Examples:
            Search the standard frequencies:
                rates = [9600, 19200, 57600, 115200, 200000, 250000, 400000, 500000, 1000000]
                resp = servos.brute_force_scan(rates)
                print(resp)

            That didn't work, try a sweep on all possible baudrates, but
            be clever and try ID 1 first:
                resp = servos.scan_baud(range(9600, 1000001, 50))
                if not resp:
                    resp = servos.brute_force_scan(range(9600, 1000001, 50))
                print(resp)

        """
        hit = False
        current_baud = self.serial_port.baudrate
        for b in bauds:
            self.serial_port.baudrate = b
            time.sleep(0.0010)
            ids = self.scan_ids()
            if ids:
                hit = b
                break
        self.serial_port.baudrate = current_baud
        return [hit, ids]
