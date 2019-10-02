
INPUT_SIZE		= 512	 # bytes

INPUT_FILE_A 	= 'hwpe_stimuli_a.hex'
INPUT_FILE_B 	= 'hwpe_stimuli_b.hex'
INPUT_FILE_C 	= 'hwpe_stimuli_c.hex'
OUTPUT_FILE_D 	= 'hwpe_stimuli_d.hex'

# signed 32-bit words from a hex file
def words_from_hex(hex_file, file_size):
	
	words = []

	for j in range(file_size/4):
		word = ''
		i = 0
		for i in range(4):
			word = str(hex_file.read(2)) + word
			# discard newline
			hex_file.read(1)
		num = int(word, 16)
		signed = num if (num < 0x80000000) else (num-0x100000000) 
		words.append(signed)

	hex_file.close()
	return words

def words_to_hex(words, hex_file):

	b = [] # bytes
	# hex = []
	for word in words:
		if word > 0x7FFFFFFF or word < -0x80000000:
			print('Warning! Number {} is going to be truncated in conversion'.format(word))
		# if negative, compute the complement and that will be the word to encode
		enc = word if word >= 0 else (0x100000000+word)
		hex_str = "%0.8x" % enc 
		for i in range(4):
			b.append(hex_str[(3-i)*2:(4-i)*2])
		# hex.append(hex_str)

	# print(hex)
	# print(b)
	with open(hex_file, 'w+') as f:
		for i in range(len(b)-1):
			f.write(b[i] + '\n')
		f.write(b[i+1])
	f.close()





def load_input_stimuli(file_a, file_b, file_c, file_size):
	
	a = []
	b = []
	c = []

	with open(file_a, 'r') as f_a:
		a = words_from_hex(f_a, file_size)
		with open(file_b, 'r') as f_b:
			b = words_from_hex(f_b, file_size)
			with open(file_c, 'r') as f_c:
				c = words_from_hex(f_c, file_size)

	return a, b, c


def compute(a, b, c, shift, simple_mult=1):
	d = []
	
	for i, a_i in enumerate(a):
		b_i = b[i]
		mul = a_i*b_i
		shifted = mul >> shift
		# print('{} * {} = {} >> {}\n'.format(a_i, b_i, mul, shifted))
		d.append(shifted)

	return d


def main():
	a, b, c = load_input_stimuli(INPUT_FILE_A, INPUT_FILE_B, INPUT_FILE_C, INPUT_SIZE)
	d = compute(a, b, c, simple_mult=1, shift=31)
	words_to_hex(d, OUTPUT_FILE_D)
	print('Done.')

if __name__ == '__main__':
	main()