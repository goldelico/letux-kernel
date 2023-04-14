
void hexdump(unsigned char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			printf("%s%08x: ", i ? "\n" : "",
					(unsigned int)&buf[i]);
		printf("%02x ", buf[i]);
	}
	printf("\n");
}
