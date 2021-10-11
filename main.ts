
let offSets: number[] = [-77, -25, 44, 145, 125, 62]; // MPU-6050
let selfCal: boolean = microbit_GY521.calibrate_Sensors(offSets);
// required to increase accuracy of readings
//
if (!selfCal) {
    basic.showLeds(`
        # . . # .
        # . # # #
        # . . # .
        # . # # #
        # . . # .
        `)
    basic.pause(30000);
}


// Motor state setup
// Use to trim motors if one is stronger than the other
//let leftMotorTrim = 0;
//let rightMotorTrim = 0;
let motorTrim = -10;
let steeringBias: number = 0;
let maxSteerBias: number = 50; //Maximum turn rate
let outMax = 1023;
let outMin = -1023;

//**************************
//****PID state setup*******
//**************************
/*
    PID algorithm
        Actuator_Output =
            Kp * (distance from goal)
        +   Ki * (accumulative error)
        +   Kd * (change in error)
        +   Ks * (current speed)
*/
let Kp = 150;  //120
let Ki = 700;  //1500
let Kd = 3;  //1.9
let Ks = 0.4;

// Manage baseline motor speed ie the value from which motor values have an affect
let motorMin = 15;  //30
let blackOut: boolean = false;
let blackOutThreshold = 30;
let setPoint = 0;
let y_Offset: number = -80;   //-80

let cycleDelay = 2;  //miliseconds
let y: number = 0;
let yTrim: number = 0;
let speed: number = 0;
let lean: number = 0;
let leanLimit: number = 0.5;   //Maximum controlled lean allowed in degrees(travel speed)
let lastInput = 0;
let lastTime = 0;
let lastOutput = 0;
let accumulative_error: number = 0;
let last_error: number = 0;
let Actuator_Output: number = 0;
let gap: number = 0;  //Absolute difference between setpoint and y

let iClamp: boolean = false  //dissable the integrator function when actuator is saturated

led.plotBarGraph(1000, 1023);
basic.pause(500);
led.plotBarGraph(750, 1023);
basic.pause(500);
led.plotBarGraph(500, 1023);
basic.pause(500);
led.plotBarGraph(250, 1023)
basic.pause(500);

basic.showLeds(`
    . # . . .
    # . . # .
    # . . . .
    # . . # .
    . # . . .
    `)

input.onButtonPressed(Button.A, () => {
    yTrim -= 0.25;
});

input.onButtonPressed(Button.B, () => {
    yTrim += 0.25;
});

let i = 0
while (i < 1000) {  //discard the first set of data from the MPU6050 to allow proper initialization
    let discard: number = 0
    discard = microbit_GY521.computeY()
    basic.pause(2)
    i++
}
led.enable(false)
while (1) {
    basic.pause(cycleDelay);  //Miliseconds
    y = microbit_GY521.computeY() + y_Offset + yTrim;


    gap = Math.abs(setPoint - y);

    if (gap > blackOutThreshold) {
        blackOut = true;
    }
    if (gap < 2) {
        blackOut = false;
    }

    if (blackOut) {
        motorStop();
    }
    else {

        speed = PID(-y);
        motorController(speed);

    }
}

function PID(input: number, ): number {

    let now = game.currentTime();    // in milliseconds
    let timeChange = (now - lastTime) / 1000;    //convert to seconds
    let distance_from_goal: number = (setPoint - input); // P
    if (iClamp) {
        accumulative_error = 0; // I (Clamped State)
    } else {
        accumulative_error = accumulative_error + (distance_from_goal * timeChange); // I (Normal State)
    }
    let change_in_error = (distance_from_goal - last_error) / timeChange; // D
    last_error = distance_from_goal;
    Actuator_Output = Kp * distance_from_goal + Ki * accumulative_error + Kd * change_in_error + Ks*lastOutput;

    let outputAbs: number = Math.abs(Actuator_Output) + motorMin;    // A base motor speed to over come motor inertia and restrict maximum power 
    if (outputAbs > outMax) {  //Saturation Check
        (outputAbs = outMax);
        if (Math.sign(distance_from_goal) == (Math.sign(Actuator_Output))) {
            //if ((distance_from_goal * Actuator_Output) > 0) {
            iClamp = true;
        }
    } else {
        iClamp = false;
    }
    if (Actuator_Output < 0) {
        Actuator_Output = -outputAbs;
    } else {
        Actuator_Output = outputAbs;
    }
    lastInput = input;
    lastTime = now;
    lastOutput = Actuator_Output

    return Actuator_Output;
}

function motorStop() {
    pins.digitalWritePin(DigitalPin.P13, 0);
    pins.digitalWritePin(DigitalPin.P14, 0);
    pins.digitalWritePin(DigitalPin.P15, 0);
    pins.digitalWritePin(DigitalPin.P16, 0);
}

function motorController(speed: number) {

    if (speed > 0) {
        pins.digitalWritePin(DigitalPin.P13, 1);
        pins.digitalWritePin(DigitalPin.P14, 0);
        pins.digitalWritePin(DigitalPin.P15, 1);
        pins.digitalWritePin(DigitalPin.P16, 0);

    }
    if (speed < 0) {

        pins.digitalWritePin(DigitalPin.P13, 0);
        pins.digitalWritePin(DigitalPin.P14, 1);
        pins.digitalWritePin(DigitalPin.P15, 0);
        pins.digitalWritePin(DigitalPin.P16, 1);
    }

    pins.analogWritePin(AnalogPin.P0, Math.abs(speed + motorTrim + steeringBias));
    pins.analogSetPeriod(AnalogPin.P0, 2500);
    pins.analogWritePin(AnalogPin.P1, Math.abs(speed - motorTrim - steeringBias));
    pins.analogSetPeriod(AnalogPin.P1, 2500);

}
