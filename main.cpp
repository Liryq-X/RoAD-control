// Exclude the min and max macros from Windows.h
#define NOMINMAX
//robot first
#include "VisionArmCombo.h"



int main(int argc, char** argv) {  //before testing water


	//int main(int /*argc*/, char* /*argv*/[]) {
	//Motor* pump = NULL;
	//pump = new Motor();F

	//for (int i=0;i<10;i++) 
	//	std::cout<<"weight: "<<pump->getWeight(3)<<std::endl;
	

	for (int i = 0; i < argc; i++)
		std::cout << "argv: " << i << ": " << argv[i] << std::endl;

	VisionArmCombo vac;
	vac.label_file_path = argv[1];
	cout << "label_file_path: " << vac.label_file_path << endl;

	/*
	cout << "test pump\n";

	Motor* pump = new Motor();

	while(1)
		cout<<"weight: "<<pump->getWeight(3);

	cout << "test finish\n";
	*/
	vac.initVisionCombo();


	//vac.getWater(300, 1);


	// PLACE_POT=1 PICK_POT=0

	//vac.placePots(PLACE_POT);
	vac.placePots(PICK_POT);
	//vac.justScan(); //if just sacn, comment initPotMap, becaue the label file confilict
	//vac.justImage();// if just image, comment initPotMap, becaue the label file confilict

	cout << "all  finished\n";
	return 0;

}
