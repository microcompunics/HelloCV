#include "../include/common.h"

std::string modelPath;

void setModelPath(std::string path){
	modelPath=path+"/CLM/";
}
std::string getModelPath(){
	return modelPath;
}

std::string getParentPath(std::string path){
	for (int i = path.size()-1; i >= 0; i--){
		if (path[i] == '/' || path[i] == '\\') return path.substr(0,i+1);
	}
	return "";
}

long getCurrentTime() {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}