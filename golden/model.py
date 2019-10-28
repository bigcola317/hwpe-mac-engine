import sys

SIMPLE_MUL		= 1
INPUT_SIZE		= 512		# bytes
VEC_LEN			= 8			# length of vectors in a and b (in 32-bit words)
SHIFT			= 31
INPUT_FILE_A 	= 'hwpe_stimuli_a.hex'
INPUT_FILE_B 	= 'hwpe_stimuli_b.hex'
INPUT_FILE_C 	= 'hwpe_stimuli_c.hex'
OUTPUT_FILE_D 	= 'hwpe_stimuli_d.hex'

STIM_PATH 		= 'stim'

# signed 32-bit words from a hex file
def words_from_hex(hex_file, file_size):
	
	words = []

	for j in range(int(file_size/4)):
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
	for j, word in enumerate(words):
		if word > 0x7FFFFFFF or word < -0x80000000:
			print('Warning! Number {} at {} is going to be truncated in conversion'.format(word, j))
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


def load_input_stimuli(file_a, file_b, file_c, size_a, size_b, size_c):
	
	a = []
	b = []
	c = []

	with open(file_a, 'r') as f_a:
		a = words_from_hex(f_a, size_a)
		with open(file_b, 'r') as f_b:
			b = words_from_hex(f_b, size_b)
			with open(file_c, 'r') as f_c:
				c = words_from_hex(f_c, size_c)

	return a, b, c

# In simple_mul mode computes element by element products of a and b.
# In MAC mode computes MAC result of vectors in a and b, that is <vec_a, vec_b> + elem_c_shifted,
# where '<,>' denotes the inner product. vec_len defines the length of each
# vector in this mode, so a and b each contain len(a)/vec_len vectors.  
# In both modes each result is right-shifted by shift.
def compute(a, b, c, shift, simple_mult, vec_len):
	d = []
	mul = 0
	acc = 0

	for i, a_i in enumerate(a):
		b_i = b[i]
		mul += a_i*b_i
		if simple_mult!=1 and i%vec_len==(vec_len-1):
			print('{}, {}, {}'.format(i, i/vec_len, len(c)))
			mul += (c[i/vec_len] << shift)
			shifted = mul >> shift
			d.append(shifted)
			# print('x + {} = {} >> {}'.format(c[i/vec_len], mul, shifted))
			mul = 0
		elif simple_mult:
			shifted = mul >> shift
			d.append(shifted)
			mul = 0

	return d


def main():
	global STIM_PATH
	a, b, c = load_input_stimuli(STIM_PATH+'/'+INPUT_FILE_A, STIM_PATH+'/'+INPUT_FILE_B, STIM_PATH+'/'+INPUT_FILE_C, INPUT_SIZE, INPUT_SIZE, INPUT_SIZE/VEC_LEN)
	print(c)
	d = compute(a, b, c, simple_mult=SIMPLE_MUL, shift=SHIFT, vec_len=VEC_LEN)
	words_to_hex(d, STIM_PATH+'/'+OUTPUT_FILE_D)
	print('Done.')


if __name__ == '__main__':
	if len(sys.argv) == 2:
		STIM_PATH = sys.argv[1]
	else:
		print('Invalid number of arguments. Usage: {} stim_path')
		sys.exit(1)
	main()
