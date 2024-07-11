#include<iostream>
#include<stdlib.h>
#include<stdio.h>
using namespace std;

int main（）
{	
	FILE* f;
	f = fopen("example.txt", "r");//How files are used? r=we can only read the file 
	if (f == NULL)
	{
		cout << "open file failed! the programmer will exit!" << endl;
	}
	char buf[1024];
	while (infile.getline(buf,sizeof(buf)))
	{
		cout << buf << endl;
	}
}
