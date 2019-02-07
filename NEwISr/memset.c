#include<stdio.h>
int main()
{
	char arr[100]="My password is 123456, please dont tell anyone";

	printf("Before memset: %s\n",arr);
	memset(arr+15,'*',6*sizeof(char));
	printf("\nAfter memset: %s\n",arr);
}
