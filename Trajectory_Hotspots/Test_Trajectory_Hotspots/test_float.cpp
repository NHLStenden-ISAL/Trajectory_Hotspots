#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/float.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestTrajectoryHotspots
{
    TEST_CLASS(TestTrajectoryHotspotsFloat)
    {
        TEST_METHOD(Construct)
        {
            Float test_float = Float(2.0f);

            Assert::IsTrue(test_float == 2.0f);
            Assert::IsTrue(test_float.get_value() == 2.0f);
        }

        TEST_METHOD(Assignment)
        {
            Float test_float;

            float f1 = 2132.f;
            Float fc1 = 12313.f;

            test_float = f1;
            Assert::IsTrue(test_float == f1);

            test_float = fc1;
            Assert::IsTrue(test_float == fc1);
        }

        TEST_METHOD(Is_Equal)
        {
            Float first_float(3.1232131f);
            Float same_float(3.1232131f);
            Float different_float(4.1231241f);

            Assert::IsTrue(first_float == same_float);
            Assert::IsFalse(first_float == different_float);
            Assert::IsTrue(first_float == first_float);
        }

        TEST_METHOD(Is_Not_Equal)
        {
            Float first_float(10.762141f);
            Float same_float(10.762141f);
            Float different_float(146546.4f);

            Assert::IsFalse(first_float != same_float);
            Assert::IsTrue(first_float != different_float);
            Assert::IsFalse(first_float != first_float);
        }

        TEST_METHOD(Is_Less)
        {
            Float first_float(10.421321f);
            Float smaller_float(3.12312f);
            Float bigger_float(20.21322f);
            Float same_float(10.421321f);

            Assert::IsFalse(bigger_float < first_float);
            Assert::IsFalse(same_float < first_float);
            Assert::IsTrue(smaller_float < first_float);

            Assert::IsFalse(first_float < 2.213213f);
            Assert::IsFalse(first_float < 10.421321f);
            Assert::IsTrue(first_float < 4103.2312f);
        }

        TEST_METHOD(Is_Greater)
        {
            Float first_float(17.081989f);
            Float smaller_float(1.21312f);
            Float bigger_float(70.f);
            Float same_float(17.081989f);

            Assert::IsTrue(bigger_float > first_float);
            Assert::IsFalse(same_float > first_float);
            Assert::IsFalse(smaller_float > first_float);

            Assert::IsTrue(first_float > 2.213213f);
            Assert::IsFalse(first_float > 17.081989f);
            Assert::IsFalse(first_float > 4103.2312f);
        }

        TEST_METHOD(Is_Less_Or_Equal)
        {
            Float first_float(152.081989f);
            Float smaller_float(11.21312f);
            Float bigger_float(420.f);
            Float same_float(152.081989f);

            Assert::IsFalse(bigger_float <= first_float);
            Assert::IsTrue(same_float <= first_float);
            Assert::IsTrue(smaller_float <= first_float);

            Assert::IsFalse(first_float <= 2.213213f);
            Assert::IsTrue(first_float <= 152.081989f);
            Assert::IsTrue(first_float <= 4103.2312f);
        }

        TEST_METHOD(Is_Greater_Or_Equal)
        {
            Float first_float(132.081989f);
            Float smaller_float(10.21312f);
            Float bigger_float(400.f);
            Float same_float(132.081989f);

            Assert::IsTrue(bigger_float >= first_float);
            Assert::IsTrue(same_float >= first_float);
            Assert::IsFalse(smaller_float >= first_float);

            Assert::IsTrue(first_float >= 2.213213f);
            Assert::IsTrue(first_float >= 132.081989f);
            Assert::IsFalse(first_float >= 4103.2312f);
        }

        TEST_METHOD(Addition)
        {
            Float fc1(2.2312f);
            Float fc2(4.123123f);

            float f1 = 2.2312f;
            float f2 = 4.123123f;

            Float sum = fc1 + fc2;

            Assert::IsTrue(sum == (f1 + f2));
            Assert::IsTrue((fc1 + f2) == (f1 + f2));

            Float fc3(12312.f);
            Float fc4(.21312f);

            float f3 = 12312.f;
            float f4 = .21312f;

            fc3 += fc4;
            f3 += f4;

            Assert::IsTrue(fc3 == f3);
        }

        TEST_METHOD(Subtraction)
        {
            Float fc1(12.2312f);
            Float fc2(3.123123f);

            float f1 = 12.2312f;
            float f2 = 3.123123f;

            Float result = fc1 - fc2;

            Assert::IsTrue(result == (f1 - f2));
            Assert::IsTrue((fc1 - f2) == (f1 - f2));

            Float fc3(83234.f);
            Float fc4(.231312f);

            float f3 = 83234.f;
            float f4 = .231312f;

            fc3 -= fc4;
            f3 -= f4;

            Assert::IsTrue(fc3 == f3);
        }

        TEST_METHOD(Multiplication)
        {
            Float fc1(3123.12f);
            Float fc2(0.002f);

            float f1 = 3123.12f;
            float f2 = 0.002f;

            Assert::IsTrue((fc1 * fc2) == (f1 * f2));
            Assert::IsTrue((fc1 * f2) == (f1 * f2));

            Float fc3(.5f);
            Float fc4(4.f);

            float f3 = 0.5;
            float f4 = 4.f;

            fc3 *= fc4;
            f3 *= f4;

            Assert::IsTrue(fc3 == f3);
        }

        TEST_METHOD(Division)
        {
            Float fc1(3123.12f);
            Float fc2(.30f);

            float f1 = 3123.12f;
            float f2 = .30f;

            Assert::IsTrue((fc1 / fc2) == (f1 / f2));
            Assert::IsTrue((fc1 / f2) == (f1 / f2));

            Float fc3(1.f);
            Float fc4(.5f);

            float f3 = 1.f;
            float f4 = .5f;

            fc3 /= fc4;
            f3 /= f4;

            Assert::IsTrue(fc3 == f3);
        }

        TEST_METHOD(Negation)
        {
            Float fc1(3123.12f);
            Float fc2(.30f);

            float f1 = 3123.12f;
            float f2 = .30f;

            Assert::IsTrue(-fc1 == -f1);
            Assert::IsTrue(-fc2 == -f2);
        }
    };
}