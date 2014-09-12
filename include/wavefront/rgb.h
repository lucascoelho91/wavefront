

#ifndef RGB_H_
#define RGB_H_

class rgb  //structure used to represent a pixel
{
    public:
        uint8_t r;
        uint8_t g;
        uint8_t b;

    rgb(uint8_t rb = 0, uint8_t gb = 0, uint8_t bb =0)
    {
    	this->r = rb;
    	this->g = gb;
    	this->b = bb;
    }

    void clear()
    {
    	r=0;
    	g=0;
    	b=0;
    }
};

#endif