using System;
using User.PluginSdkDemo;

namespace DiyFfbPlugin.Tests
{
    internal static class Program
    {
        private static int Main()
        {
            int failures = 0;
            int total = 0;

            RunTest("CenteredContactZero", TestCenteredContactZero, ref total, ref failures);
            RunTest("MissingContactThrows", TestMissingContactThrows, ref total, ref failures);
            RunTest("CollinearBarSolves", TestCollinearBarSolves, ref total, ref failures);
            RunTest("NegativeTravelThrows", TestNegativeTravelThrows, ref total, ref failures);
            RunTest("DuplicatePinIdThrows", TestDuplicatePinIdThrows, ref total, ref failures);
            RunTest("ContactGroundedThrows", TestContactGroundedThrows, ref total, ref failures);
            RunTest("RailGroundedThrows", TestRailGroundedThrows, ref total, ref failures);
            RunTest("ContactRailSameThrows", TestContactRailSameThrows, ref total, ref failures);
            RunTest("MeteringBarPinCountThrows", TestMeteringBarPinCountThrows, ref total, ref failures);
            RunTest("MultipleMeteringBarsThrows", TestMultipleMeteringBarsThrows, ref total, ref failures);
            RunTest("MissingMeteringThrows", TestMissingMeteringThrows, ref total, ref failures);
            RunTest("NonCollinearBarThrows", TestNonCollinearBarThrows, ref total, ref failures);
            RunTest("SharedCollinearBarThrows", TestSharedCollinearBarThrows, ref total, ref failures);
            RunTest("CoefficientsFinite", TestCoefficientsFinite, ref total, ref failures);

            Console.WriteLine($"Tests run: {total}, Failures: {failures}");
            return failures == 0 ? 0 : 1;
        }

        private static void RunTest(string name, Action test, ref int total, ref int failures)
        {
            total++;
            try
            {
                test();
                Console.WriteLine($"[PASS] {name}");
            }
            catch (Exception ex)
            {
                failures++;
                Console.WriteLine($"[FAIL] {name}: {ex.Message}");
                Console.WriteLine(ex);
            }
        }

        private static void TestCenteredContactZero()
        {
            const double travelNegative = 15.0;
            const double travelPositive = 25.0;
            global::GeneralKinematicConfig config = BuildTriangleConfig(travelNegative, travelPositive);
            global::KinematicParameters parameters = GeneralKinematics.CalcKinematicParameters(config);

            AssertTrue(parameters.CoeffsSledPosOverContactPointPos.Count > 0, "Missing sled position coefficients.");
            double sledAtZero = parameters.CoeffsSledPosOverContactPointPos[0];
            AssertNear(travelNegative, sledAtZero, 1.0, "Contact position zero should map to rail center.");

            AssertTrue(parameters.ContactPointPosMinAbs < 0, "ContactPointPosMinAbs should be negative after centering.");
            AssertTrue(parameters.ContactPointPosMaxAbs > 0, "ContactPointPosMaxAbs should be positive after centering.");
        }

        private static void TestMissingContactThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 10.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            bar.PinIds.Add(1);
            bar.PinIds.Add(2);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Missing contact point should throw an ArgumentException.");
        }

        private static void TestCollinearBarSolves()
        {
            global::GeneralKinematicConfig config = BuildCollinearConfig(15.0, 25.0);
            global::KinematicParameters parameters = GeneralKinematics.CalcKinematicParameters(config);

            AssertTrue(parameters.CoeffsSledPosOverContactPointPos.Count > 0,
                "Missing sled position coefficients for collinear bar.");
            AssertTrue(parameters.ContactPointPosMinAbs < 0,
                "ContactPointPosMinAbs should be negative after centering.");
            AssertTrue(parameters.ContactPointPosMaxAbs > 0,
                "ContactPointPosMaxAbs should be positive after centering.");
        }

        private static void TestNegativeTravelThrows()
        {
            global::GeneralKinematicConfig config = BuildTriangleConfig(-1.0, 5.0);
            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Negative rail travel should throw an ArgumentException.");
        }

        private static void TestDuplicatePinIdThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 50.0f,
                Y = 50.0f,
                IsContactPoint = true
            });
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            bar.PinIds.Add(1);
            bar.PinIds.Add(2);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Duplicate pin_id should throw an ArgumentException.");
        }

        private static void TestContactGroundedThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true,
                IsContactPoint = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            bar.PinIds.Add(1);
            bar.PinIds.Add(2);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Grounded contact pin should throw an ArgumentException.");
        }

        private static void TestRailGroundedThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 50.0f,
                Y = 50.0f,
                IsContactPoint = true
            });
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            bar.PinIds.Add(1);
            bar.PinIds.Add(2);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Grounded rail pin should throw an ArgumentException.");
        }

        private static void TestContactRailSameThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                IsContactPoint = true,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 50.0f,
                Y = 50.0f,
                Grounded = true
            });
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            bar.PinIds.Add(1);
            bar.PinIds.Add(2);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Contact point pin cannot be rail interface pin.");
        }

        private static void TestMeteringBarPinCountThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 3,
                X = 50.0f,
                Y = 50.0f,
                IsContactPoint = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 4,
                X = 25.0f,
                Y = 25.0f
            });
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            bar.PinIds.Add(1);
            bar.PinIds.Add(2);
            bar.PinIds.Add(3);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Metering bar with more than two pins should throw an ArgumentException.");
        }

        private static void TestMultipleMeteringBarsThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 3,
                X = 50.0f,
                Y = 50.0f,
                IsContactPoint = true
            });
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            bar.PinIds.Add(1);
            bar.PinIds.Add(2);
            config.Bars.Add(bar);
            bar = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            bar.PinIds.Add(2);
            bar.PinIds.Add(3);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Multiple metering bars should throw an ArgumentException.");
        }

        private static void TestMissingMeteringThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 3,
                X = 50.0f,
                Y = 50.0f,
                IsContactPoint = true
            });
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar();
            bar.PinIds.Add(1);
            bar.PinIds.Add(3);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Missing metering bar should throw an ArgumentException.");
        }

        private static void TestNonCollinearBarThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 3,
                X = 50.0f,
                Y = 50.0f,
                IsContactPoint = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 4,
                X = 60.0f,
                Y = 10.0f
            });
            global::GeneralKinematicBar metering = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            metering.PinIds.Add(2);
            metering.PinIds.Add(3);
            config.Bars.Add(metering);
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar();
            bar.PinIds.Add(1);
            bar.PinIds.Add(3);
            bar.PinIds.Add(4);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Non-collinear bar should throw an ArgumentException.");
        }

        private static void TestSharedCollinearBarThrows()
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = 5.0f,
                RailTravelPositive = 5.0f
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 3,
                X = 50.0f,
                Y = 0.0f
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 4,
                X = 150.0f,
                Y = 0.0f
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 5,
                X = 200.0f,
                Y = 0.0f,
                IsContactPoint = true
            });
            global::GeneralKinematicBar metering = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            metering.PinIds.Add(2);
            metering.PinIds.Add(5);
            config.Bars.Add(metering);
            global::GeneralKinematicBar bar = new global::GeneralKinematicBar();
            bar.PinIds.Add(1);
            bar.PinIds.Add(3);
            bar.PinIds.Add(4);
            config.Bars.Add(bar);
            bar = new global::GeneralKinematicBar();
            bar.PinIds.Add(3);
            bar.PinIds.Add(4);
            bar.PinIds.Add(5);
            config.Bars.Add(bar);

            AssertThrows<ArgumentException>(() => GeneralKinematics.CalcKinematicParameters(config),
                "Pins in multiple collinear bars should throw an ArgumentException.");
        }

        private static void TestCoefficientsFinite()
        {
            global::GeneralKinematicConfig config = BuildTriangleConfig(10.0, 10.0);
            global::KinematicParameters parameters = GeneralKinematics.CalcKinematicParameters(config);
            AssertFinite(parameters.CoeffsSledPosOverContactPointPos, "Sled coefficients contain NaN/Infinity.");
            AssertFinite(parameters.CoeffsForceFactorOverContactPointPos, "Force coefficients contain NaN/Infinity.");
        }

        private static global::GeneralKinematicConfig BuildTriangleConfig(double travelNegative, double travelPositive)
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = (float)travelNegative,
                RailTravelPositive = (float)travelPositive
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 3,
                X = 50.0f,
                Y = 50.0f,
                IsContactPoint = true
            });

            global::GeneralKinematicBar metering = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            metering.PinIds.Add(2);
            metering.PinIds.Add(3);
            config.Bars.Add(metering);

            global::GeneralKinematicBar link = new global::GeneralKinematicBar();
            link.PinIds.Add(1);
            link.PinIds.Add(3);
            config.Bars.Add(link);

            return config;
        }

        private static global::GeneralKinematicConfig BuildCollinearConfig(double travelNegative, double travelPositive)
        {
            global::GeneralKinematicConfig config = new global::GeneralKinematicConfig
            {
                RailTravelNegative = (float)travelNegative,
                RailTravelPositive = (float)travelPositive
            };
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 2,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 3,
                X = 50.0f,
                Y = 50.0f
            });
            config.Pins.Add(new global::GeneralKinematicPin
            {
                PinId = 4,
                X = 100.0f,
                Y = 100.0f,
                IsContactPoint = true
            });

            global::GeneralKinematicBar metering = new global::GeneralKinematicBar
            {
                IsMetering = true
            };
            metering.PinIds.Add(2);
            metering.PinIds.Add(3);
            config.Bars.Add(metering);

            global::GeneralKinematicBar collinear = new global::GeneralKinematicBar();
            collinear.PinIds.Add(1);
            collinear.PinIds.Add(3);
            collinear.PinIds.Add(4);
            config.Bars.Add(collinear);

            return config;
        }

        private static void AssertTrue(bool condition, string message)
        {
            if (!condition)
            {
                throw new InvalidOperationException(message);
            }
        }

        private static void AssertNear(double expected, double actual, double tolerance, string message)
        {
            if (Math.Abs(expected - actual) > tolerance)
            {
                throw new InvalidOperationException($"{message} Expected {expected:F3}, got {actual:F3}.");
            }
        }

        private static void AssertThrows<TException>(Action action, string message) where TException : Exception
        {
            try
            {
                action();
            }
            catch (TException)
            {
                return;
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"{message} Threw {ex.GetType().Name} instead of {typeof(TException).Name}.");
            }

            throw new InvalidOperationException($"{message} No exception was thrown.");
        }

        private static void AssertFinite(System.Collections.Generic.IEnumerable<double> values, string message)
        {
            foreach (double value in values)
            {
                if (double.IsNaN(value) || double.IsInfinity(value))
                {
                    throw new InvalidOperationException(message);
                }
            }
        }
    }
}
