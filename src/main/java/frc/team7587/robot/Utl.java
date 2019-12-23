
package frc.team7587.robot;

import java.time.LocalDateTime;

public final class Utl {
    private static long ts = System.currentTimeMillis();

    public static void log(String s){
        // 
        long millis = System.currentTimeMillis();
        if( (millis - ts) > 150){
            System.out.println("{" + LocalDateTime.now() + "} " + s);
            ts = millis;
        }
    }

    public static void log0(String s){
        // 
            System.out.println("{" + LocalDateTime.now() + "} " + s);
    }


}