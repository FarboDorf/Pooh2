#include <iostream>
#include <cmath>

using namespace std;


class PIDImpl
{
public:
	// Kp - proportional gain
	// Ki - Integral gain
	// Kd - derivative gain
	// dt - loop interval time
	// max - maximum value of manipulated variable
	// min - minimum value of manipulated variable
	PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki);
	~PIDImpl();
	// Returns the manipulated variable given a setpoint and current process value
	double calculate(double setpoint, double pv);

private:
	double _dt;
	double _max;
	double _min;
	double _Kp;
	double _Kd;
	double _Ki;
	double _pre_error;
	double _integral;
};



/**
* Implementation
*/
PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki) :
	_dt(dt),
	_max(max),
	_min(min),
	_Kp(Kp),
	_Kd(Kd),
	_Ki(Ki),
	_pre_error(0),
	_integral(0)
{
}

double PIDImpl::calculate(double setpoint, double pv) {

	// Calculate error
	double error = setpoint - pv;

	// Proportional term
	double Pout = _Kp * error;

	// Integral term
	_integral += error * _dt;
	double Iout = _Ki * _integral;

	// Derivative term
	double derivative = (error - _pre_error) / _dt;
	double Dout = _Kd * derivative;

	// Calculate total output
	double output = Pout + Iout + Dout;

	//std::cout<<"out="<<output<< "Pout = "<< Pout <<" " <<Iout <<" "<< Dout <<std::endl;
	// Restrict to max/min
	if (output > _max)
		output = _max;
	else if (output < _min)
		output = _min;

	std::cout << "outB=" << output << std::endl;

	// Save error to previous error
	_pre_error = error;

	return output;
}

PIDImpl::~PIDImpl()
{
}


enum ECmd {
	ENone,
	EExit
};

int counter = 0;

ECmd getCmd() {
	counter++;
	if (counter >= 500) {
		return EExit;
	}
	else
		return ENone;
}

float getHZadFromUser() {
	return 10.;
}


class Bear {
	float h;
	float m;
	float v;
	float a;
public:
	Bear() {
		v = 0; // m/s
		a = 0; // m/s2
		m = 10; // kg
		h = 0; // Текущая высота
	}
	float getH() {
		return h;
	}
	float getV() {
		return v;
	}
	float getM() {
		return m;
	}
	float getA() {
		return a;
	}
	void eat(float weightFood) {
		m += weightFood;
	}
	virtual void calc(float dt, float extrnForce) {
		float dh;
		v = v + a * dt;
		dh = v * dt;
		h = h + dh;
		a = (extrnForce - 9.8 * m) / m; // m * a = mg - F

		if (h < 0) { // Stay on the ground
			h = 0.;
			v = 0.;
			a = 0.;
		}
	}

	virtual void show() {
		std::cout << "h=" << h << std::endl;
	}
};

class Engine;



class Engine {
	float enginePower = 0; // n 0..500 n
	float maxPower; // n
public:
	Engine(float imaxPower) {
		enginePower = 0.;
		maxPower = imaxPower;
	}

	void setPower(float enginePowerPercent)
	{
		if (enginePowerPercent > 100.)
			enginePowerPercent = 100;
		if (enginePowerPercent < -100.)
			enginePowerPercent = -100;

		enginePower = enginePowerPercent * (maxPower / 100.);
	}

	float getForce() {
		return enginePower;
	}
};

class SmartFlyingBear :public Bear {
	PIDImpl* pid;
	Engine* pengine;
	float hZad;
public:
	SmartFlyingBear(PIDImpl* ppid, Engine* ppengine) {
		pid = ppid;
		pengine = ppengine;
		hZad = 0;
	}

	void setHZad(float val) {
		hZad = val;
	}

	virtual void calc(float dt, float extrnForce)
	{
		float enginePowerPercent = 0;
		if (pid != nullptr) {
			enginePowerPercent = pid->calculate(hZad, getH());
		}

		if (pengine != nullptr) {
			pengine->setPower(enginePowerPercent);
			Bear::calc(dt, extrnForce + pengine->getForce());
		}
	}
};


int main() {

	PIDImpl pid(0.1, 100, -100, 0.1, 0.01, 0.1);
	Bear pooh;
	SmartFlyingBear superpooh(new PIDImpl(0.1, 100, -100, 1.5, 1.28, 0.45), new Engine(500.));

	float t = 0; // Время моделирования
	float enginePower = 0; // n 0..500 n
	float enginePowerPercent = 0; // 0..100
	float deltaEnginePowerPercent = 0;
	float dt = 0.1; // s

	float hZad = 0; // Измененение высоты за шаг моделирования

	ECmd cmd = ENone;


	while (cmd != EExit) {
		cmd = getCmd(); // Неблокирующее получение команды


		hZad = getHZadFromUser(); // Неблокирующая функция для получения высоты

		superpooh.setHZad(hZad);
		superpooh.show();
		superpooh.calc(dt, 0);

		if ((abs(superpooh.getH() - hZad) <= 1) && (int(t) % 5 > 3)) {
			superpooh.eat(0.1);
		}

		t = t + dt;

		// std::cout << "power = " << Engine.getForce() << std::endl;
		cout << "mass = " << superpooh.getM() << endl;
	}

	return 0;
}