/** 
 * TEAM CAPTCHA FALL 2018
 * Joseph Primmer   | Vicente Caycedo
 * Eldor Bekpulatov | Francis Rayos del Sol
 * 
 * Deprecated code.
 */

void figureEight(){
  if(figure8State < 4){
    turnLeft();
    figure8State++;
  }else{
    turnRight();
    figure8State++;
  }
  if(figure8State > 7){
    figure8State = 0;
  }
  topState = 0;
  
}

