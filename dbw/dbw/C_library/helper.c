#include <stdio.h>

int square(int i)
{
	return i*i;
}

struct Sumup
{
	int num1;
	int num2;
};

int mysum(struct Sumup num)
{
	return num.num1 + num.num2;
}