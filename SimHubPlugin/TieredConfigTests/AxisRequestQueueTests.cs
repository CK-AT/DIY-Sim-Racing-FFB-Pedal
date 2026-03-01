using System;
using System.Collections.Generic;
using ProtbufTest;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    public static class AxisRequestQueueTests
    {
        private class MockSender : IAxisRequestSender
        {
            public List<(AxisID AxisId, AxisRequestType Type, Message Payload)> Sent
                = new List<(AxisID, AxisRequestType, Message)>();
            public bool SendResult = true;

            public bool SendAxisRequest(AxisID axisId, AxisRequestType type, Message payload)
            {
                Sent.Add((axisId, type, payload));
                return SendResult;
            }
        }

        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                TestRunner.RunTest("UploadWithVerify_EnqueuesQuery",
                    UploadWithVerify_EnqueuesQuery),
                TestRunner.RunTest("UploadWithoutVerify_NoQuery",
                    UploadWithoutVerify_NoQuery),
                TestRunner.RunTest("FunctionConfigUploadVerify_EnqueuesFunctionConfigQuery",
                    FunctionConfigUploadVerify_EnqueuesFunctionConfigQuery),
                TestRunner.RunTest("AxisConfigUploadVerify_EnqueuesAxisConfigQuery",
                    AxisConfigUploadVerify_EnqueuesAxisConfigQuery),
                TestRunner.RunTest("VerifySkippedForBroadcast",
                    VerifySkippedForBroadcast),
                TestRunner.RunTest("VerifyDeduplicatesExistingQuery",
                    VerifyDeduplicatesExistingQuery),
                TestRunner.RunTest("VerifyFlagIgnoredForNonUpload",
                    VerifyFlagIgnoredForNonUpload),
            };
        }

        private static void UploadWithVerify_EnqueuesQuery()
        {
            var sender = new MockSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);
            var msg = new Message { AxisConfig = new AxisConfig { AxisId = AxisID._1 } };

            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfigUpload, msg,
                verifyAfterSend: true);

            // Process the upload
            queue.Tick();

            // Upload was sent
            if (sender.Sent.Count != 1)
                throw new Exception($"Expected 1 send, got {sender.Sent.Count}");
            if (sender.Sent[0].Type != AxisRequestType.AxisConfigUpload)
                throw new Exception("Expected AxisConfigUpload");

            // Verification query should be enqueued
            if (queue.QueueCount != 1)
                throw new Exception($"Expected 1 queued item (verify query), got {queue.QueueCount}");

            // Advance past cooldown
            var baseTime = DateTime.UtcNow.AddSeconds(1);
            queue.SetNowProvider(() => baseTime);
            queue.Tick();

            // Verification query was sent
            if (sender.Sent.Count != 2)
                throw new Exception($"Expected 2 sends, got {sender.Sent.Count}");
            if (sender.Sent[1].Type != AxisRequestType.AxisConfig)
                throw new Exception($"Expected AxisConfig query, got {sender.Sent[1].Type}");
            if (sender.Sent[1].AxisId != AxisID._1)
                throw new Exception($"Expected axis _1, got {sender.Sent[1].AxisId}");
        }

        private static void UploadWithoutVerify_NoQuery()
        {
            var sender = new MockSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);
            var msg = new Message { AxisConfig = new AxisConfig { AxisId = AxisID._1 } };

            queue.Enqueue(AxisID._1, AxisRequestType.AxisConfigUpload, msg);

            queue.Tick();

            if (sender.Sent.Count != 1)
                throw new Exception($"Expected 1 send, got {sender.Sent.Count}");
            if (queue.QueueCount != 0)
                throw new Exception($"Expected empty queue, got {queue.QueueCount}");
        }

        private static void FunctionConfigUploadVerify_EnqueuesFunctionConfigQuery()
        {
            var sender = new MockSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);
            var msg = new Message { FunctionConfig = new FunctionConfig() };

            queue.Enqueue(AxisID._2, AxisRequestType.FunctionConfigUpload, msg,
                verifyAfterSend: true);

            queue.Tick();

            if (queue.QueueCount != 1)
                throw new Exception($"Expected 1 queued verify, got {queue.QueueCount}");

            var baseTime = DateTime.UtcNow.AddSeconds(1);
            queue.SetNowProvider(() => baseTime);
            queue.Tick();

            if (sender.Sent.Count != 2)
                throw new Exception($"Expected 2 sends, got {sender.Sent.Count}");
            if (sender.Sent[1].Type != AxisRequestType.FunctionConfig)
                throw new Exception($"Expected FunctionConfig query, got {sender.Sent[1].Type}");
            if (sender.Sent[1].AxisId != AxisID._2)
                throw new Exception($"Expected axis _2, got {sender.Sent[1].AxisId}");
        }

        private static void AxisConfigUploadVerify_EnqueuesAxisConfigQuery()
        {
            var sender = new MockSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);
            var msg = new Message { AxisConfig = new AxisConfig { AxisId = AxisID._3 } };

            queue.Enqueue(AxisID._3, AxisRequestType.AxisConfigUpload, msg,
                verifyAfterSend: true);

            queue.Tick();

            if (queue.QueueCount != 1)
                throw new Exception($"Expected 1 queued verify, got {queue.QueueCount}");

            var baseTime = DateTime.UtcNow.AddSeconds(1);
            queue.SetNowProvider(() => baseTime);
            queue.Tick();

            if (sender.Sent.Count != 2)
                throw new Exception($"Expected 2 sends, got {sender.Sent.Count}");
            if (sender.Sent[1].Type != AxisRequestType.AxisConfig)
                throw new Exception($"Expected AxisConfig query, got {sender.Sent[1].Type}");
            if (sender.Sent[1].AxisId != AxisID._3)
                throw new Exception($"Expected axis _3, got {sender.Sent[1].AxisId}");
        }

        private static void VerifySkippedForBroadcast()
        {
            var sender = new MockSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);
            var msg = new Message { FunctionConfig = new FunctionConfig() };

            // Broadcast upload (AxisUndefined) — can't verify because ESP32 won't respond
            queue.Enqueue(AxisID.AxisUndefined, AxisRequestType.FunctionConfigUpload, msg,
                verifyAfterSend: true);

            queue.Tick();

            if (sender.Sent.Count != 1)
                throw new Exception($"Expected 1 send, got {sender.Sent.Count}");
            // No verification query should be enqueued for AxisUndefined
            if (queue.QueueCount != 0)
                throw new Exception($"Expected empty queue (no verify for broadcast), got {queue.QueueCount}");
        }

        private static void VerifyDeduplicatesExistingQuery()
        {
            var sender = new MockSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);

            // Pre-enqueue a FunctionConfig query for axis _1
            queue.Enqueue(AxisID._1, AxisRequestType.FunctionConfig);

            // Now enqueue an upload with verify for same axis
            var msg = new Message { FunctionConfig = new FunctionConfig() };
            queue.Enqueue(AxisID._1, AxisRequestType.FunctionConfigUpload, msg,
                verifyAfterSend: true);

            // Queue should have: query + upload = 2 items
            if (queue.QueueCount != 2)
                throw new Exception($"Expected 2 queued items, got {queue.QueueCount}");

            // Process the FunctionConfig query (it awaits response, so simulate response)
            queue.Tick();
            // Respond to clear the query
            queue.HandleResponse(new Message { FunctionConfig = new FunctionConfig() });

            // Now process the upload
            var baseTime = DateTime.UtcNow.AddSeconds(1);
            queue.SetNowProvider(() => baseTime);
            queue.Tick();

            // Verification query should be enqueued (no existing duplicate blocks it)
            if (queue.QueueCount != 1)
                throw new Exception($"Expected 1 queued verify, got {queue.QueueCount}");
        }

        private static void VerifyFlagIgnoredForNonUpload()
        {
            var sender = new MockSender();
            var queue = new AxisRequestQueue(sender, manualTick: true);

            // verifyAfterSend on a non-upload type should be ignored
            queue.Enqueue(AxisID._1, AxisRequestType.Restart, null, verifyAfterSend: true);

            queue.Tick();

            if (sender.Sent.Count != 1)
                throw new Exception($"Expected 1 send, got {sender.Sent.Count}");
            if (queue.QueueCount != 0)
                throw new Exception($"Expected empty queue, got {queue.QueueCount}");
        }
    }
}
