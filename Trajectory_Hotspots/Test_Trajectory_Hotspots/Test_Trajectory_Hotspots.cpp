#include "pch.h"
#include "CppUnitTest.h"
//TODO: Fix these ugly cpp includes?
#include "../Trajectory_Hotspots/Trajectory_Hotspots.cpp"
#include "../Trajectory_Hotspots/vec2.cpp"
#include "../Trajectory_Hotspots/utils.cpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestTrajectoryHotspots
{

    TEST_CLASS(TestTrajectoryHotspotsVec2)
    {
    public:

        TEST_METHOD(vector_length)
        {
            Vec2 a(0.0f, 1.0f);
            Vec2 b(1.0f, 0.f);

            float length_a = a.length();
            Assert::IsTrue(nearly_equal(length_a, 1.0f));

            Vec2 c = a + b;

            float length_c = c.length();

            Assert::IsTrue(nearly_equal(length_c, 1.4142135623731f));
        }

        TEST_METHOD(vector_normalize)
        {
            Vec2 a(1, 1);

            a.normalize();

            float a_length = a.length();
            Assert::IsTrue(nearly_equal(a_length, 1.0f));


            Assert::IsTrue(nearly_equal(a.x, 1.f / sqrtf(2.0f)));
            Assert::IsTrue(nearly_equal(a.y, 1.f / sqrtf(2.0f)));
        }

        TEST_METHOD(vector_dot)
        {
            Vec2 a(2.f, 3.f);
            Vec2 b(-4.f, 3.f);

            float dotAB = a.dot(b);
            Assert::IsTrue(nearly_equal(dotAB, 1.0f));

            Vec2 c(2.f, 3.f);

            float dotAC = a.dot(c);
            float dotCA = c.dot(a);
            Assert::IsTrue(nearly_equal(dotAC, 13.0f));
            Assert::IsTrue(nearly_equal(dotCA, 13.0f));

            
            Assert::IsTrue(nearly_equal(a.normalized().dot(c.normalized()), 1.0f));

            Vec2 neg_c = -c;

            float dot_negC_A = neg_c.dot(c);
            Assert::IsTrue(nearly_equal(dot_negC_A, -13.0f));


            Vec2 a_big(82723.f, 78343.f);
            Vec2 b_big(48943.f, 1880880.f);
            
            Assert::IsTrue(nearly_equal(a_big.dot(b_big), 151402493629.0f));
            std::cout << a_big.dot(b_big) << std::endl;


            Vec2 a_low(.0082723f, .0078343f);
            Vec2 b_low(.0048943f, .1880880f);
            
            Assert::IsTrue(nearly_equal(a_low.dot(b_low), 0.001514024936290000188088f));
        }

        TEST_METHOD(vector_negative)
        {
            Vec2 a(3123.f, 31232.f);

            Vec2 a_neg = -a;

            a_neg.x = -3123.f;
            a_neg.y = -31232.f;
        }

        TEST_METHOD(vector_subtract)
        {
            Vec2 a(0.00032f, 3.232f);
            Vec2 b(1.0f, 1232.f);

            Vec2 c = a - b;

            Assert::IsTrue(nearly_equal(c.x, -0.9996799f));
            Assert::IsTrue(nearly_equal(c.y, -1228.76790f));
        }

        TEST_METHOD(vector_add)
        {
            Vec2 a(0.2367f, 0.0006f);
            Vec2 b(2381.2f, 832.232f);

            Vec2 c = a + b;

            Assert::IsTrue(nearly_equal(c.x, 2381.4367f));
            Assert::IsTrue(nearly_equal(c.y, 832.23260f));
        }
    };
}
