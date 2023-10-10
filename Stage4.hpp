/* 
MIT License

Copyright (c) 2023 Huang I Chun

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Prediction

#ifndef STAGE4_HPP
#define STAGE4_HPP

class Stage4_Point_Based : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud

public:
    Stage4_Point_Based(int my_mode);
    void run(Pipeline_Object &pipeline_obj);
};

class Stage4_Cube_Based : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud
    int iterate_round;

public:
    Stage4_Cube_Based(int my_mode, int my_iterate_round);
    void run(Pipeline_Object &pipeline_obj);
};

class Stage4_Neighboring_Based : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud

public:
    Stage4_Neighboring_Based(int my_mode);
    void run(Pipeline_Object &pipeline_obj);
};
#endif