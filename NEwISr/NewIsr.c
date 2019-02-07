#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<float.h>

void reverseString(char str[5]);
void main()
{
	char InputBuffer[3],lsb,msb;
	int ConvertStringIntoInt=0;
	float ConvertIntToTimeInMilisec=0;
	unsigned long long int Pulse=0,NeedPulse=0,CompleteClock =65535;
	float deno = 10.0;
	float clockPerCycle=0.25;//microsecinds
	int remainder=0; 
	char HexlevelBuffer[5];
	int i=0;
	char a,b;

//	printf(" ***********Exapmle************\n");
//	printf("*    input 10                 *\n");
//	printf("*    MSB = 1 and LSB = 0      *\n");
//	printf("********************************\n\n");
//	printf("Enter the msb and then lsb\n");
	scanf("%c%c",&msb,&lsb);
//	printf("MSB:%c and LSB : %c\n",msb,lsb);
	InputBuffer[1]=lsb;
	InputBuffer[0]=msb;	
	
	ConvertStringIntoInt = atoi(InputBuffer);
//	printf("Output in interger is %d\n",ConvertStringIntoInt);
	ConvertIntToTimeInMilisec = (ConvertStringIntoInt/deno);
//	printf("time in miliseconds is %.1f\n",ConvertIntToTimeInMilisec);
	
//	printf("********calculate the required pu6lses**************\n");
	ConvertIntToTimeInMilisec = (ConvertIntToTimeInMilisec*1000);//convert into microseconds
	Pulse = (ConvertIntToTimeInMilisec/clockPerCycle);
//	printf("pulses are: %llu\n",Pulse);
	NeedPulse = CompleteClock - Pulse;//65535-pulse
//	printf("Need pulses are %llu\n",NeedPulse);
	sprintf(HexlevelBuffer,"%X",NeedPulse);
	printf("Hex value is %s\n",HexlevelBuffer);
	reverseString(HexlevelBuffer);
	
}

void reverseString(char str[5]){
	int i,j;
	int start=0;
	int end = strlen(str)-1;
	char temp,strH[3],strL[3];
	int TimerH,TimerL;
	
	strncpy(strH,str,2);
	strH[2]='\0';
	printf("Higer Nibble: %s",strH);
	printf("\n");
	strncpy(strL,str+2,2);
	strL[2]='\0';
	printf("Lower Nibble: %s",strL);
	//printf("reverse string is %s",str);
//	while(start <= (end))
//	{
//		if(start <= 1)
//		{
//				temp = str[start];
//				strH[start]=str[end-2];
//				strH[end-2]=temp;
//				start++;
//				end--;
//		}
//		else if(start >= 2 && start < 4)
//		{
//			    temp = str[start];
//				strL[start]=strL[end-2];
//				strL[end-2]=temp;
//				start++;
//	          	end--;
//		}
//
//	}
	
//	printf("Hex value is %s\n",strH);
//	printf("Hex value is %s\n",strL);
	 
//	 for(i=0,j=0;i<strlen(str);i++,j+=2)
//    { 
//        sprintf((char*)strH+j,"%X",str[i]);
//    }
//    strH[j]='\0'; /*adding NULL in the end*/
//    printf("Hexadecimal converted string is: ");
//    printf("%s\n",strH);
}

