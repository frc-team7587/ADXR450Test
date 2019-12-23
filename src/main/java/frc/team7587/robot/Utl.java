
package frc.team7587.robot;

import java.time.LocalDateTime;

public final class Utl {
    private static long ts = System.currentTimeMillis();

    public static void log(String s){
        // 
        long millis = System.currentTimeMillis();
        if( (millis - ts) > 100){
            System.out.println("{" + LocalDateTime.now() + "} " + s);
            ts = millis;
        }
    }

    public static void log0(String s){
        // 
            System.out.println("{" + LocalDateTime.now() + "} " + s);
    }

    // public static void log(String s, double millisCount){
    //     long millis = System.currentTimeMillis();
    //     if( (millis - ts) > 500){
    //         log(s);
    //         ts = millis;
    //     }
    // }
}