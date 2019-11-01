#include "camera.h"
using namespace std;
int main()
{
	Camera camera("camera_01");
	camera.printInformation();
	camera.getDisparityMap("camera_01");
	getchar();
	return 0 ;
}