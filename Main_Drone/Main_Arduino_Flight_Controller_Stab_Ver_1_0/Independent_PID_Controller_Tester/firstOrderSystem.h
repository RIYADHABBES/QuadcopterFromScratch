
class FirstOrderSystem {
private:
    double gain;
    double timeConstant;
    double initialPoint;
    double long timeStep;
    double long time;
public:
    FirstOrderSystem(double k, double tau,float initPoint) {
        gain = k;
        timeConstant = tau;
        initialPoint = initPoint;
        timeStep = micros() / 1000.0;
        time = micros() / 1000.0;
    }

    double computeOutput(double input) {
        double output = gain * input + initialPoint;

        output = input * gain * (1 - exp(-timeStep / timeConstant));

        timeStep = (micros() / 1000.0) - time;
        time = micros() / 1000.0;
        return output;
    }
};