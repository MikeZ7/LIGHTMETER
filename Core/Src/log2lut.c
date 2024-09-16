#include "log2lut.h"

int log_2_lut(int value){
	if(value == 0 || value == 1){
		return 0;
	}
	else if(value == 2){
		return 1;
	}
	else if(value > 2 && value < 6){
		return 2;
	}
	else if(value > 5 && value < 12){
		return 3;
	}
	else if(value > 11 && value < 23){
		return 4;
	}
	else if(value > 22 && value < 46){
		return 5;
	}
	else if(value > 45 && value < 91){
		return 6;
	}
	else if(value > 90 && value < 182){
		return 7;
	}
	else if(value > 181 && value < 363){
		return 8;
	}
	else if(value > 362 && value < 725){
		return 9;
	}
	else if(value > 724 && value < 1449){
		return 10;
	}
	else if(value > 1448 && value < 2897){
		return 11;
	}
	else if(value > 2896 && value < 5793){
		return 12;
	}
	else if(value > 5792 && value < 11586){
		return 13;
	}
	else if(value > 11585 && value < 23171){
		return 14;
	}
	else if(value > 23170 && value < 46341){
		return 15;
	}
	else if(value > 46340 && value < 56001){
		return 16;
	}
	else{
		return 2137;
	}
}
