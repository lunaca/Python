#/usr/bin/python

#RSA Assymetric Cipher Encryption Project 2
#Zac Brooks Allen Courtway



import random, math, os, sys
DEFBLOCKSIZE = 128
BYTESIZE = 256

def main():
	message = input('Please type the secret message you would like me to cipher and decipher good sir!>>>>>>>')# user can make a message 
	keySize =1024
	
	inputfilename = input('Please give me a name for the text file you would like to write message encryption in!(without .txt) >>>>>>>') # the file to write to/read from
	writefilename = "".join((inputfilename, '.txt'))
	
	filename= input('Please give me a name for the text file you would like to put your keys in! (without.txt) >>>>>>>') # the filename for the public and private keys
	
	
	
	writeKeytoTxtFile(filename, 340)
	print(' your key files are written to ', filename, '.txt')
	publicKeyFileName = "".join((filename, '_pubkey.txt'))
	privateKeyFileName= "".join((filename, '_privkey.txt'))
	print(publicKeyFileName)
	
	print(' Your keys are saved in' , publicKeyFileName, 'and ', privateKeyFileName, '!')
	
	

	print('Encrypting and writing to %s...' % (writefilename))
	encryptedText = encryptAndWrite(writefilename, publicKeyFileName, message)
	
	print('Encrypted text:')
	print(encryptedText)
	print('Decrypting and writing to %s....' % (writefilename))
	decryptedText = readAndDecrypt(writefilename, privateKeyFileName)
	print('Decrypted text:')
	print(decryptedText)






#Functions that make this all possible:)




def encryptAndWrite(messageFilename, keyFilename, message, blockSize=DEFBLOCKSIZE):
	
	#using the key from a file, this function encrypts the message and returns an encrypted message by the PUBLIC KEY
	
	keySize, n, e = readKeyFile(keyFilename)
		
	# Encrypt the message
	eBlocks = encryption(message, (n, e), blockSize)

	# Convert the large int values to one string value.
	for i in range(len(eBlocks)):
		eBlocks[i] = str(eBlocks[i])
	encryptedContent = ','.join(eBlocks)

 	# Write out the encrypted string to the output file.
	encryptedContent = '%s_%s_%s' % (len(message), blockSize, encryptedContent)
	fo = open(messageFilename, 'w')
	fo.write(encryptedContent)
	fo.close()
	# Also return the encrypted string.
	return encryptedContent
	
def readAndDecrypt(messageFilename, keyFilename):

	#this function reads in the encrypted message and cracks it with the PRIVATE key and using Blocks to Text function
	keySize, n, d = readKeyFile(keyFilename)


	#Read in the message length and the encrypted message from the file.
	fo = open(messageFilename)
	content = fo.read()
	
	messageLength, blockSize, encryptedMessage = content.split('_')
	messageLength = int(messageLength)
	blockSize = int(blockSize)
	
	# Convert the encrypted message into large int values.
	eBlocks = []
	for block in encryptedMessage.split(','):
		eBlocks.append(int(block))

	# Decrypt the large int values using decryption
	return decryption(eBlocks, messageLength, (n, d), blockSize)
	
def encryption(message, key, blockSize=DEFBLOCKSIZE):

	#this function converts the message string into a list full of block integers
	eBlocks = []
	n, e = key

	for block in textToBlocks(message, blockSize):
 		# ciphertext = plaintext ^ e mod n
		eBlocks.append(pow(block, e, n))
	return eBlocks
	
def decryption(eBlocks, messageLength, key, blockSize=DEFBLOCKSIZE):

	#this function takes a list of encrypted blocks and decrypts it back into the original message string using get TextFromBlocks
	decryptedBlocks = []
	n, d = key
	for block in eBlocks:
		# plaintext = ciphertext ^ d mod n
		decryptedBlocks.append(pow(block, d, n))
	
	return blocksToText(decryptedBlocks, messageLength, blockSize)



def readKeyFile(keyFilename):
	# Given the filename of a file that contains a public or private key,
	# return the key as a (n,e) or (n,d) tuple value.
	fo = open(keyFilename)
	content = fo.read()
	fo.close()
	keySize, n, EorD = content.split(',')
	return (int(keySize), int(n), int(EorD))

def textToBlocks(message, blockSize=DEFBLOCKSIZE):
	# Converts a string message to a list of block integers. Each integer
	# represents 128 (or whatever blockSize is set to) string characters.
	messageBytes = message.encode('ascii') # convert the string to bytes

	blockInts = []
	for blockStart in range(0, len(messageBytes), blockSize):
		# Calculate the block integer for this block of text
		blockInt = 0
		for i in range(blockStart, min(blockStart + blockSize, len(messageBytes))):
			blockInt += messageBytes[i] * (BYTESIZE ** (i % blockSize))
			blockInts.append(blockInt)
	
	return blockInts
	
def blocksToText(blockInts, messageLength, blockSize=DEFBLOCKSIZE):
	 # Converts a list of block integers to the original message string.
	 # The original message length is needed to properly convert the last
	# block integer.
	message = []
	totalmessage = ''
	count = 0
	for blockInt in blockInts:
		blockMessage = []
		previousmessageblock = ''
		
		for i in range(blockSize - 1, -1, -1):
			
			if len(message) + i < messageLength*128:
	                 # Decode the message string for the 128 (or whatever
	                 # blockSize is set to) characters from this block integer.
				asciiNumber = blockInt // (BYTESIZE ** i)
				blockInt = blockInt % (BYTESIZE ** i)
				blockMessage.insert(0, chr(asciiNumber))
				
		message.extend(blockMessage)
		
		count += 1
		if count == 128:
			previousmessageblock = ''.join(blockMessage)
			
			totalmessage = ''.join((previousmessageblock,totalmessage))
			
			count = 0
	
	leftovermessage = ''.join(blockMessage)
	
	finaltotalmessage = ''.join((totalmessage, leftovermessage))
	
	return (finaltotalmessage)
	

## this simply writes our keys to text files, a long with our D AND E values


def writeKeytoTxtFile(name, keySize):


	# Our safety check will prevent us from overwriting our old key files

	if os.path.exists('%s_pubkey.txt' % (name)) or os.path.exists('%s_privkey.txt' % (name)):
		sys.exit('WARNING: The file %s_pubkey.txt or %s_privkey.txt already exists! Use a different name or delete these files and re-run this program.' % (name, name))

	publicKey, privateKey = generateKey(keySize)

	
	fo = open('%s_pubkey.txt' % (name), 'w')
	fo.write('%s,%s,%s' % (keySize, publicKey[0], publicKey[1]))
	fo.close()

	
	fo = open('%s_privkey.txt' % (name), 'w')
	fo.write('%s,%s,%s' % (keySize, privateKey[0], privateKey[1]))
	fo.close()
	
# this function will generate OUR AWESOME PRIVATE AND PUBLIC KEYS \/

def generateKey(keySize):
	#this function will generate a public and private key pair 
	
	
	# Step 1: First we shall create two prime numbers, p and q. Calculate n = p * q.
	print('Generating p prime...')
	p = meMakeBigPrime(keySize)
	
	print('Generating q prime...')
	q = meMakeBigPrime(keySize)
	print ('our second prime is', q)
	n = p * q
	
	# Step 2: Create a number e that is relatively prime to (p-1)*(q-1).
	print('Generating e that is relatively prime to (p-1)*(q-1)...')
	while True:
	# Keep trying random numbers for e until one is valid.
		e = random.randrange(2 ** (keySize - 1), 2 ** (keySize))
		if math.gcd(e, (p - 1) * (q - 1)) == 1:
			break
	
	# Step 3: Calculate d, the mod inverse of e.
	print('Calculating d that is mod inverse of e...')
	d = modinv(e, (p - 1) * (q - 1))

	publicKey = (n, e)
	privateKey = (n, d)

	#print('Public key:', publicKey)
	#print('Private key:', privateKey)

	return (publicKey, privateKey)


#this is a popular algorithm that generates a prime number that we will later check \/


def rabinMillerAlg(num):
	# Returns True if num is a prime number.

	s = num - 1
	t = 0
	while s % 2 == 0:
		# keep halving s while it is even (and use t
		# to count how many times we halve s)
		s = s // 2
		t += 1

	for trials in range(5): # try to falsify num's primality 5 times
		a = random.randrange(2, num - 1)
		v = pow(a, s, num)
		if v != 1: # this test does not apply if v is 1.
			i = 0
			while v != (num - 1):
				if i == t - 1:
					return False
				else:
					i = i + 1
					v = (v ** 2) % num
	return True


#this tests if a number is prime using a couple quicker steps, and if that fails, we use the rabinMiller algorithm. \/

def isPrime(num):
	# Return True if num is a prime number. This function does a quicker
	# prime number check before calling rabinMiller().

	if (num < 2):
		return False # 0, 1, and negative numbers are not prime

	# About 1/3 of the time we can quickly determine if num is not prime
	# by dividing by the first few dozen prime numbers. This is quicker
	# than rabinMiller(), but unlike rabinMiller() is not guaranteed to
	# prove that a number is prime.
	lowPrimes = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271, 277, 281, 283, 293, 307, 311, 313, 317, 331, 337, 347, 349, 353, 359, 367, 373, 379, 383, 389, 397, 401, 409, 419, 421, 431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487, 491, 499, 503, 509, 521, 523, 541, 547, 557, 563, 569, 571, 577, 587, 593, 599, 601, 607, 613, 617, 619, 631, 641, 643, 647, 653, 659, 661, 673, 677, 683, 691, 701, 709, 719, 727, 733, 739, 743, 751, 757, 761, 769, 773, 787, 797, 809, 811, 821, 823, 827, 829, 839, 853, 857, 859, 863, 877, 881, 883, 887, 907, 911, 919, 929, 937, 941, 947, 953, 967, 971, 977, 983, 991, 997]

	if num in lowPrimes:
		return True

	# See if any of the low prime numbers can divide num
	for prime in lowPrimes:
		if (num % prime == 0):
			return False

	# If all else fails, call rabinMiller() to determine if num is a prime.
	return rabinMillerAlg(num)
	


#this will be called to help us generate random prime numbers for the key \/

def meMakeBigPrime(keysize):
	# Return a random prime number of keysize bits in size.
	while True:
		num = random.randrange(2**(keysize-1), 2**(keysize))
		if isPrime(num):
			print (num)
			return num

#this will help us find the modular inverse of e for the keys \/

def egcd(a, b):
	if a == 0:
		return (b, 0, 1)
	else:
		g, y, x = egcd(b % a, a)
		return (g, x - (b // a) * y, y)

def modinv(a, m):
	g, x, y = egcd(a, m)
	if g != 1:
		raise Exception('modular inverse does not exist')
	else:
		return x % m

main()