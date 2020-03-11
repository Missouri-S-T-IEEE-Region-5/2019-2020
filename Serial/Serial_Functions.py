import struct



# Start function definitions

#TODO: Figure out if we need bit string or bit list for output

'''
Input: 
    bit_list (either bit string or bit list)
Returns:
    byte_list = currently bit string
'''
def bit_to_byte_list(bit_list):
    # ALWAYS: casts the bit list to int
    bit_list = list(map(int, bit_list))

    # READ: Depends on if the bit list needs to be reversed
    reversed_bit_list = [bit for bit in reversed(bit_list)]
    length_bit_list = len(bit_list)
    if length_bit_list % 8 != 0:
        zeroes = 8 - (length_bit_list % 8)
        zeroes_list = [0 for zero in range(zeroes)]
        reversed_bit_list = zeroes_list + reversed_bit_list

    counter = 8
    last = 0
    length_bit_list = len(reversed_bit_list)
    byte_list = []

    '''
    Turns [1,1,1,0] to '1110' then casts to int
    Casts int to byte
    Appends byte to byte list
    '''
    while (counter <= length_bit_list):
        byte_int = int(''.join(map(str, reversed_bit_list[last:counter])), 2)
        byte = bytes([byte_int])
        byte_list.append(byte)
        last = counter
        counter += 8

    # READ: DEPENDS ON IF IT NEEDS A BYTE STRING OR LIST
    byte_list = b''.join(byte_list)
    return byte_list


'''
    Input: 
        byte_list = list of 4 bytes
        type = character for type conversion:
            'f': signed float
            'i': signed integer 
            'I': unsigned integer
    Returns:
        val = value of unpacked float or int
        
'''
def unpack_bytes(byte_list,type):

    # sees if list, converts to bit string
    if isinstance(byte_list, list):
        byte_list = b''.join(byte_list)

    val = struct.unpack(type, byte_list)

    return val


'''
    Input: 
        val  = value of float or int
        type = character for type conversion:
            'f': signed float
            'i': signed integer 
            'I': unsigned integer
    Returns:
        float or integer value
'''
def pack_val(val, type):
    # Converts to makes sure either float or int
    if type == 'i' or type == 'I':
        val = int(val)
    else:
        val = float(val)

    byte_list = struct.pack(type, val)
    return byte_list


# End function definitions
