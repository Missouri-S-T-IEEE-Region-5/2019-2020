from Serial_Functions import unpack_bytes, pack_val, bit_to_byte_list

'''TESTING CODE TO ASSURE FUNCTIONALITY'''

# TEST PACKING
packing_chars = ['i', 'f', 'I']  # i = integer, f = float, I = unsigned integer
packing_list = [-1, 1.1, 1]  # test values for packing into 4 bytes
packed_list = []  # for storing packed byte date

# PACKS VALUES (by looping through and sending to pack_val function)
for val, type_needed in zip(packing_list, packing_chars):
    packed_list.append(pack_val(val, type_needed))


# Create bit list to turn into val of choice (4 bytes)
bit_list = [1 for i in range(32)]  # bit list to turn into byte list (should be 4294967295 for all ones)
type_for_bit_list = 'I'  # Type to turn bit list into see types defined above
byte_list = bit_to_byte_list(bit_list)  # Converts bit list to byte list
packed_list.append(byte_list)  # Appends byte for random generated to packed list
packing_chars.append(type_for_bit_list) # Appends type for the packed bit list

print(byte_list)  # prints byte list

# Prints unpacked values
for packed_val, type_needed in zip(packed_list, packing_chars):
    print(unpack_bytes(packed_val, type_needed))
