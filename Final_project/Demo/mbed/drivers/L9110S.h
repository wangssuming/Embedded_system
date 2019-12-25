#include "mbed.h"

/** L9110S H-Brigh Driver Interface
 *
 * By Sebastian Donner
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * @code
 * #include "mbed.h"
 * #include "L9110S.h"
 *
 * Serial debug(USBTX,USBRX);
 * L9119S motor_x(p21, p22);
 *
 * int main() 
 * { 
 *
 *  int power = 0;
 *  int step  = 1;
   
 *    debug.baud(115200);
 *    debug.printf("L9110S Motor Test");
 *    
 *    while(1) 
 *    {
 *      power += step;
 *      motor_x.drive(power);
 *      debug.printf("Power: %i\n\r", power );
 *      wait(0.10);
 *      if ((power > 99)||(power < -99)) step *= -1;
 *    }
 * }
 * @endcode
 */
class L9110S {
    public:
        //! Min Power in PWM% for clockwise rotate
        int cw_min;

        //! Max Power in PWM% for clockwise rotate 
        int cw_max;
        
        //! Min Power in PWM% for counter clockwise rotate
        int ccw_min;
        
        //! Max Power in PWM% for counter clockwise rotate
        int ccw_max;
        
        
        /** Create a new interface for an L9110S
         *
         * @param cw_r  is the pin for clockwise rotate
         * @param ccw_r is the pin for counter clockwise rotate
         */
        L9110S(PinName cw_r, PinName ccw_r);

        /** sets Pulswidth cw(+) ccw(-)
         * 
         * @param power positiv in clockwise Dir
         * @param power negativ in counter clockwise Dir
         *
         *  in % of range from min_(cw/ccw) to max_(cw/ccw)
         *
         */
        void drive(int power);
        
        /** sets Pulswidth and Dir 
         * 
         *  @param dir positiv dir is clockwise direction
         *  @param dir negativ dir is counter clockwise direction
         *  @param dir zero is stop 
         *
         *  @param power in % of range from min_(cw/ccw) to max_(cw/ccw) allways positiv
         *
         */
        void drive(int dir, int power);      
         
        /** sets the PWM frequency
         *
         * @param hz is the PWM frequency (default 2000 Hz)
         */
        void frequency(int hz);
    
        void drive_diff(float Soll, float Ist, int kp ,int i_lim, int trash);
        /** sets the PWM frequency
         *
         * @param Soll  target value
         * @param Ist   actual value
         * @param kp    prop Faktor
         * @param i_lim I Limiter 
         * @param trash Deathband wide in value * 10
         *
         */
    
    private:
        PwmOut _cw;
        PwmOut _ccw;
        int periode;
        int power_act;
        float drive_i;
        float deg_diff(float Soll, float Ist);
        
};