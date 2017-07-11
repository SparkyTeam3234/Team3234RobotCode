class GyroCorrect
{
	float targetAngle;
	float tol = 1;
	float ltol=0;
	float utol=0;
	float crrtvl=0;
	float k = 0.005; //correction scalar
public:
	void setTargetAngle(float currentAngle) {targetAngle=currentAngle;}
	float getTargetAngle() {return targetAngle;}
	void setTol(float tolin) {tol=tolin;}
	float goTargetAngle(float currentAngle) {
		ltol = targetAngle - tol;
		utol = targetAngle +tol;
		if (ltol <= currentAngle && currentAngle <= utol)
		{
			crrtvl=0;
		}
		else
		{
			crrtvl = k*(targetAngle-currentAngle);
		}
		return crrtvl;
		/*
		signed int wAngle=0;
		if (tol>=0)
		{
		if (((int) currentAngle)>((int) targetAngle)+tol)
		{
			wAngle=-1;
		}
		else if (((int) currentAngle)<((int)targetAngle-tol))
		{
			wAngle=1;
		}
		else
		{
			wAngle=0;
		};

	}
		else
		{
			wAngle=0;
		}
		return wAngle;
	*/
	}
};
