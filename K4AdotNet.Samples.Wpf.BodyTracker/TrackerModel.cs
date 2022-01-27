﻿using K4AdotNet.BodyTracking;
using K4AdotNet.Sensor;
using System;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Collections.Generic;

using System.IO;
using Newtonsoft.Json;

namespace K4AdotNet.Samples.Wpf.BodyTracker
{
    internal sealed class TrackerModel : ViewModelBase, IDisposable
    {
        private readonly Calibration calibration;
        private readonly BackgroundReadingLoop readingLoop;
        private readonly BackgroundTrackingLoop trackingLoop;

        // To visualize images received from Capture
        private readonly ImageVisualizer colorImageVisualizer;
        private readonly ImageVisualizer depthImageVisualizer;
        // To visualize skeletons
        private readonly SkeletonVisualizer depthSkeletonVisualizer;
        private readonly SkeletonVisualizer colorSkeletonVisualizer;
        // To transform body index map from depth camera to color camera
        private readonly BodyIndexMapTransformation bodyIndexMapTransformation;

        private readonly ActualFpsCalculator actualFps = new ActualFpsCalculator();

        private int status_json=0;



        // For designer
        public TrackerModel()
            : base()
        {
            Title = "Kinect for Azure #123456";
            DepthColumnWidth = ColorColumnWidth = new GridLength(1, GridUnitType.Star);
        }

        public TrackerModel(IApp app, BackgroundReadingLoop readingLoop,
            TrackerProcessingMode processingMode, DnnModel dnnModel, SensorOrientation sensorOrientation, float smoothingFactor)
            : base(app)
        {

            // try to create tracking loop first
            readingLoop.GetCalibration(out calibration);
            trackingLoop = new BackgroundTrackingLoop(in calibration, processingMode, dnnModel, sensorOrientation, smoothingFactor);
            trackingLoop.BodyFrameReady += TrackingLoop_BodyFrameReady;
            trackingLoop.Failed += BackgroundLoop_Failed;

            this.readingLoop = readingLoop;
            readingLoop.CaptureReady += ReadingLoop_CaptureReady;
            readingLoop.Failed += BackgroundLoop_Failed;

            Title = readingLoop.ToString();

            // Image and skeleton visualizers for depth
            var depthMode = readingLoop.DepthMode;
            depthImageVisualizer = ImageVisualizer.CreateForDepth(dispatcher, depthMode.WidthPixels(), depthMode.HeightPixels());
            depthSkeletonVisualizer = new SkeletonVisualizer(dispatcher, depthMode.WidthPixels(), depthMode.HeightPixels(), ProjectJointToDepthMap);

            // Image and skeleton visualizers for color
            var colorRes = readingLoop.ColorResolution;
            if (colorRes != ColorResolution.Off)
            {
                colorImageVisualizer = ImageVisualizer.CreateForColorBgra(dispatcher, colorRes.WidthPixels(), colorRes.HeightPixels());
                colorSkeletonVisualizer = new SkeletonVisualizer(dispatcher, colorRes.WidthPixels(), colorRes.HeightPixels(), ProjectJointToColorImage);
                bodyIndexMapTransformation = new BodyIndexMapTransformation(in calibration);
            }

            // Proportions between columns
            if (colorRes != ColorResolution.Off)
            {
                DepthColumnWidth = new GridLength(depthImageVisualizer.WidthPixels, GridUnitType.Star);
                ColorColumnWidth = new GridLength(
                    depthImageVisualizer.HeightPixels * colorImageVisualizer.WidthPixels / colorImageVisualizer.HeightPixels,
                    GridUnitType.Star);
            }
            else
            {
                DepthColumnWidth = new GridLength(1, GridUnitType.Star);
                ColorColumnWidth = new GridLength(0, GridUnitType.Pixel);
            }
        }

        private Float2? ProjectJointToDepthMap(Joint joint)
            => calibration.Convert3DTo2D(joint.PositionMm, CalibrationGeometry.Depth, CalibrationGeometry.Depth);

        private Float2? ProjectJointToColorImage(Joint joint)
            => calibration.Convert3DTo2D(joint.PositionMm, CalibrationGeometry.Depth, CalibrationGeometry.Color);

        private void BackgroundLoop_Failed(object sender, FailedEventArgs e)
            => dispatcher.BeginInvoke(new Action(() => app.ShowErrorMessage(e.Exception.Message)));

        private void ReadingLoop_CaptureReady(object sender, CaptureReadyEventArgs e)
            => trackingLoop.Enqueue(e.Capture);

        private void TrackingLoop_BodyFrameReady(object sender, BodyFrameReadyEventArgs e)
        {
            
            if (actualFps.RegisterFrame())
                RaisePropertyChanged(nameof(ActualFps));

            var data = depthSkeletonVisualizer?.Update(e.BodyFrame, status_json);
            colorSkeletonVisualizer?.Update(e.BodyFrame, status_json);

            var end_time = DateTime.Now;

            if ((end_time - start_time).TotalMilliseconds> 200 && data !=null) {
                bodyDataProcessed.Add(data);
                start_time= DateTime.Now;
            }

            using (var capture = e.BodyFrame.Capture)
            {
                using (var depthImage = capture.DepthImage)
                {
                    using (var indexMap = e.BodyFrame.BodyIndexMap)
                    {
                        depthImageVisualizer?.Update(depthImage, indexMap);

                        if (colorImageVisualizer != null)
                        {
                            using (var colorImage = capture.ColorImage)
                            {
                                using (var transformedBodyIndexMap = bodyIndexMapTransformation.ToColor(depthImage, indexMap))
                                {
                                    colorImageVisualizer.Update(colorImage, transformedBodyIndexMap);
                                }
                            }
                        }

                    }
                }
            }
            if (status_json == 1)
            {
                ImportJSON(bodyDataProcessed);
            }
        }

        public List<BodyImport> bodyDataProcessed = new List<BodyImport>();

        public DateTime start_time=DateTime.Now;

        public void ImportJSON(List<BodyImport> dataProcessed)
        {

            Console.WriteLine("Json crear");
            string data_json = JsonConvert.SerializeObject(dataProcessed);

            string exp_name="Azure_" + DateTime.Now.ToString("yyyy-MM-dd")+"_"+ DateTime.Now.ToString("HH-mm");

            string path = Path.Combine(exp_name+".json");

            using (StreamWriter writer = new StreamWriter(path))
            {
                writer.Write(data_json);
            }

            status_json =0;
        }
        public void Dispose()
        {
            if (readingLoop != null)
            {
                readingLoop.Failed -= BackgroundLoop_Failed;
                readingLoop.CaptureReady -= ReadingLoop_CaptureReady;
                readingLoop.Dispose();
            }

            if (trackingLoop != null)
            {
                trackingLoop.Failed -= BackgroundLoop_Failed;
                trackingLoop.BodyFrameReady -= TrackingLoop_BodyFrameReady;
                trackingLoop.Dispose();
            }

            bodyIndexMapTransformation?.Dispose();
        }

        public void Import()
        {
            status_json = status_json+1;
            Console.WriteLine("Estatus Iniciado");
        }


        public void Run()
            => readingLoop?.Run();

        public string Title { get; }

        public BitmapSource ColorImageSource => colorImageVisualizer?.ImageSource;
        public BitmapSource DepthImageSource => depthImageVisualizer?.ImageSource;

        public ImageSource DepthSkeletonImageSource => depthSkeletonVisualizer?.ImageSource;
        public ImageSource ColorSkeletonImageSource => colorSkeletonVisualizer?.ImageSource;

        public byte DepthNonBodyPixelsAlpha
        {
            get => depthImageVisualizer?.NonBodyAlphaValue ?? byte.MaxValue;
            set
            {
                if (value != DepthNonBodyPixelsAlpha && depthImageVisualizer != null)
                {
                    depthImageVisualizer.NonBodyAlphaValue = value;
                    RaisePropertyChanged(nameof(DepthNonBodyPixelsAlpha));
                }
            }
        }

        public byte ColorNonBodyPixelsAlpha
        {
            get => colorImageVisualizer?.NonBodyAlphaValue ?? byte.MaxValue;
            set
            {
                if (value != ColorNonBodyPixelsAlpha && colorImageVisualizer != null)
                {
                    colorImageVisualizer.NonBodyAlphaValue = value;
                    RaisePropertyChanged(nameof(ColorNonBodyPixelsAlpha));
                }
            }
        }

        public string ActualFps => FormatFps(actualFps);

        private static string FormatFps(ActualFpsCalculator fps)
        {
            var value = fps.FramesPerSecond;
            return value > float.Epsilon ? value.ToString("0.0") : string.Empty;
        }

        public GridLength DepthColumnWidth { get; }
        public GridLength ColorColumnWidth { get; }
    }
}
