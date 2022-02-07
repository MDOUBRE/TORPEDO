#include <stdio.h>




int get_error(short int * array){

	unsigned char TB = 0b00000000;
	unsigned char TBT;
	for (short int i =0 ; i < 8; i++){
		TB = TB << 1;
		printf("%d\n", array[i]);
		if (array[i]> 6){
			TB = TB | 0b00000001;
		}
		
	}
	printf("%u\n",TB);
	if (!(TB ^ 0b00011000))
	{
		return 0;
	}else if(!(TB ^ 0b00001000))
	{
		return 1;
	}else if(!(TB ^ 0b00001100))
	{
		return 2;
	}else if(!(TB ^ 0b00000100))
	{
		return 3;
	}else if(!(TB ^ 0b00000110))
	{
		return 4;
	}else if(!(TB ^ 0b00000010))
	{
		return 5;
	}else if(!(TB ^ 0b00000011))
	{
		return 6;
	}else if(!(TB ^ 0b00000001))
	{
		return 7;
	}else if(!(TB ^ 0b00010000))
	{
		return -1;
	}else if(!(TB ^ 0b00110000))
	{
		return -2;
	}else if(!(TB ^ 0b00100000))
	{
		return -3;
	}else if(!(TB ^ 0b01100000))
	{
		return -4;
	}else if(!(TB ^ 0b01000000))
	{
		return -5;
	}else if(!(TB ^ 0b11000000))
	{
		return -6;
	}else if(!(TB ^ 0b10000000))
	{
		return -7;
	}else if(!(TB ^ 0b00000000))
	{
		return 10;
	}else
	{
		TBT = (TB & 0b11111000)
		if(!(TBT ^ 0b11111000) || !(TBT ^ 0b11110000) || !(TBT ^ 0b11101000) || !(TBT ^ 0b11011000) || !(TBT ^ 0b10111000) || !(TBT ^ 0b01111000)) 
		{
			TBT = (TB & 0b00011111)
			if(!(TBT ^ 0b00011111) || !(TBT ^ 0b00011110) || !(TBT ^ 0b00011101) || !(TBT ^ 0b00011011) || !(TBT ^ 0b00010111) || !(TBT ^ 0b000001111)) 
			{
				return 11
			}
			return 12
		}else 
		{
			TBT = (TB & 0b00011111)
			if(!(TBT ^ 0b00011111) || !(TBT ^ 0b00011110) || !(TBT ^ 0b00011101) || !(TBT ^ 0b00011011) || !(TBT ^ 0b00010111) || !(TBT ^ 0b000001111)) 
			{
				return 13
			}
		}
	}
	return 0;
}



int main() {
	short int vc[8] = { 1 , 1 , 1 , 1 , 12 , 1 , 1 , 1 };
	int a;
	a = get_error(vc);
	printf("%d\n",a);
	return 0;
}
 
