#ifndef _RECEIVER_H_                                                                                            
#define _RECEIVER_H_                                                                                            
                                                                                                                
class Receiver                                                                                                  
{                                                                                                               
public :                                                                                                        
	unsigned long InputThrottle, InputYaw, InputPitch, InputRoll;                                                 
                                                                                                                
	Receiver();                                                                                                   
	~Receiver();                                                                                                  
                                                                                                                
	void Init();                                                                                                  
	void UpdateInput(unsigned long throttle, unsigned long rudder, unsigned long elevator, unsigned long aileron);
                                                                                                                
private :                                                                                                       
                                                                                                                
};                                                                                                              
                                                                                                                
#endif                                                                                                          