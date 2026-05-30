#ifndef PID_H
#define PID_H

class pid
{
    float K ,b, Ti, h, I, Tt, y_old;
    int antiWindup, bumpless, feedback;
    float Kold, bold;

public:
    explicit pid(float _K = 0.2, float _b = 1, float _Ti = 0.1, float _h = 10e-3); // Added default params for h, b, Ti
    ~pid() {};
    float computeControl(float r, float y);
    float saturate(float value, float min_val, float max_val);
    void setK(float newK);
    float getK();
    void setB(float newB);
    float getB();
    void setTi(float newTi);
    void setAntiWindup(int value);
    int getAntiWindup();
    void setFeedback(int value);
    int getFeedback();
    void setBumpless (int value);
    int getBumpless ();
};



#endif