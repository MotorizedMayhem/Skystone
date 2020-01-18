package org.firstinspires.ftc.teamcode.MM_Classes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class MM_IterativeOpMode extends OpMode {
    public MecanumYellow robot = new MecanumYellow();

    @Override
    public void init(){
        robot.init(hardwareMap);
        telemetry.addData("Hardware Status", "Robot started");
        telemetry.update();
    }





    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided sleep() method.
     *
     * @param ms amount of time to sleep, in milliseconds
     */
    public final void sleep(int ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    /**
     * Puts the current thread to sleep for a bit as it has nothing better to do. This allows other
     * threads in the system to run.
     *
     * <p>One can use this method when you have nothing better to do in your code as you await state
     * managed by other threads to change. Calling idle() is entirely optional: it just helps make
     * the system a little more responsive and a little more efficient.</p>
     */
    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }
}
