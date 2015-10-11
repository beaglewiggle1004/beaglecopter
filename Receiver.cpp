#include "Receiver.h"                                                   
                                                                        
using namespace std;                                                    
                                                                        
Receiver::Receiver() {                                                  
                                                                        
}                                                                       
                                                                        
Receiver::~Receiver() {                                                 
                                                                        
}                                                                       
                                                                        
void Receiver::Init() {                                                 
	InputThrottle = 0;                                                    
	InputYaw = 512;                                                       
	InputPitch = 512;                                                     
	InputRoll = 512;                                                      
}                                                                       
                                                                        
void Receiver::UpdateInput(unsigned long throttle, unsigned long rudder,
		unsigned long elevator, unsigned long aileron) {                    
	InputThrottle = throttle;                                             
	InputYaw = rudder;                                                    
	InputPitch = elevator;                                                
	InputRoll = aileron;                                                  
}                                                                       