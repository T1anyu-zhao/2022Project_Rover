bool start = 1;
  if (start){   
    int headroom = 150; // prevent hitting the wall
    //int increment = 300; in declearation

    float length_r = 200, width_r = 200; //size of the rover[mm]
    float length_c = 3600, width_c = 2400; //size of the court

    period =  (width_c-2*headroom) / (3*width_r);
    //bool go_straight_1 = false, go_straight_2 = false, go_straight_3 = false, go_straight_4 = true;
    //bool turn_90_1 = false, turn_90_2 = false, turn_90_3 = false, turn_90_4 = false;

    for(i; i <= period; i++){


      if(!go_straight_1 && go_straight_4){

        mode = 'C';
        destination_x = increment;
        destination_y = length_c-(500+length_r);
        if(reachDestination){
          go_straight_1 = true;
          reachDestination = false;
        }
      }

      else if(go_straight_1 && !go_straight_2){
        
        increment = increment + 300;
        mode = 'C';
        destination_x = increment;
        destination_y = length_c-(500+length_r);
        if(reachDestination){
          go_straight_2 = true;
          reachDestination = false;
        }
      }

      else if(go_straight_2 && !go_straight_3){
        
        mode = 'C';
        destination_x = increment;
        destination_y = 500+length_r;
        if(reachDestination){
          go_straight_3 = true;
          reachDestination = false;
        }
      }

      else if(go_straight_3 && !go_straight_4){

        if(i = period){
          mode = 'C';
          destination_x = 300;
          destination_y = 300;
          if(reachDestination){
            go_straight_4 = true;
            cout << "route finished" << endl;
            return;
          }
        }

        increment = increment + 300;
        mode = 'C';
        destination_x = 200+increment;
        destination_y = headroom+length_r;
        if(reachDestination){
          go_straight_4 = true;
          reachDestination = false;
          go_straight_1 = false;
        }
        
      }

      
  }
