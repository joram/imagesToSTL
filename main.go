package main

import (
	"C"
	"fmt"
	"github.com/davecgh/go-spew/spew"
	"gocv.io/x/gocv"
	"os"
	"path/filepath"
	"strings"
)

func readImagesFromDirectory(dirPath string) ([]*gocv.Mat, error) {
	images := []*gocv.Mat{}

	err := filepath.Walk(dirPath, func(path string, info os.FileInfo, err error) error {
		if err != nil {
			return err
		}

		// Check if the file is an image (you can customize this check based on your image file extensions).
		if !info.IsDir() && _isImageFile(path) {
			img := gocv.IMRead(path, gocv.IMReadColor)
			if img.Empty() {
				fmt.Printf("Failed to read image: %s\n", path)
			} else {
				images = append(images, &img)
			}
		}

		return nil
	})

	if err != nil {
		return nil, err
	}

	return images, nil
}

func _isImageFile(filename string) bool {
	// You can customize this function to check for specific image file extensions.
	ext := strings.ToLower(filepath.Ext(filename))
	return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" || ext == ".gif"
}

func featureMatching(images []*gocv.Mat) ([][]gocv.DMatch, []gocv.KeyPoint, []gocv.KeyPoint, []gocv.Mat, error) {
	if len(images) < 2 {
		return nil, nil, nil, nil, fmt.Errorf("At least two images are required for feature matching")
	}

	// Create a SIFT detector
	sift := gocv.NewSIFT()
	defer sift.Close()

	// Create a Brute-Force Matcher for feature matching
	bf := gocv.NewBFMatcher()
	defer bf.Close()

	// Initialize the matches, keypoints, and camera matrices
	matches := make([][]gocv.DMatch, len(images)-1)
	keypoints1 := make([]gocv.KeyPoint, len(images)-1)
	keypoints2 := make([]gocv.KeyPoint, len(images)-1)
	cameras := make([]gocv.Mat, len(images)-1)

	for i := 0; i < len(images)-1; i++ {
		img1 := images[i]
		img2 := images[i+1]

		// Detect keypoints and compute descriptors for both images
		kp1, desc1 := sift.DetectAndCompute(*img1, gocv.NewMat())
		kp2, desc2 := sift.DetectAndCompute(*img2, gocv.NewMat())

		// Match the descriptors
		results := bf.Match(desc1, desc2)
		matches[i] = results

		// Filter matches based on a distance threshold (adjust as needed)
		goodMatches := make([]gocv.DMatch, 0)
		for _, match := range matches[i] {
			if match.Distance < 0.75 { // Adjust the threshold as needed
				goodMatches = append(goodMatches, match)
			}
		}

		matches[i] = goodMatches
		keypoints1[i] = kp1[0] // ??? wrong (added [0])
		keypoints2[i] = kp2[0] // ??? wrong (added [0])

		// Assuming you have camera matrices (camMatrix) for each image, add them to the cameras slice.
		camMatrix := gocv.NewMat()
		cameras[i] = camMatrix
	}

	return matches, keypoints1, keypoints2, cameras, nil
}

//func triangulatePoints(matches []gocv.DMatch, keypoints1, keypoints2 []gocv.KeyPoint, cameras []gocv.Mat) ([]gocv.Point3f, error) {
//	if len(matches) == 0 || len(keypoints1) == 0 || len(keypoints2) == 0 || len(cameras) != 2 {
//		return nil, fmt.Errorf("Invalid input data for triangulation")
//	}
//
//	// Create the 3D points container
//	points3D := make([]gocv.Point3f, 0)
//
//	// Retrieve the camera matrices
//	cameraMatrix1 := cameras[0]
//	cameraMatrix2 := cameras[1]
//
//	for _, match := range matches {
//		point1X, point1Y := keypoints1[match.QueryIdx].X, keypoints1[match.QueryIdx].Y
//		point2X, point2Y := keypoints2[match.TrainIdx].X, keypoints2[match.TrainIdx].Y
//
//		// Convert 2D points to homogeneous coordinates
//		point1Homogeneous, err := gocv.NewMatFromBytes(3, 1, gocv.MatType(gocv.MatTypeCV32F), []byte{
//			byte(float32(point1X)), byte(float32(point1Y)), 1.0,
//		})
//		if err != nil {
//			return nil, err
//		}
//
//		point2Homogeneous, err := gocv.NewMatFromBytes(3, 1, gocv.MatType(gocv.MatTypeCV32F), []byte{
//			byte(float32(point2X)), byte(float32(point2Y)), 1.0,
//		})
//		if err != nil {
//			return nil, err
//		}
//
//		// Perform triangulation
//		point3D := gocv.NewMat()
//		opencv.TriangulatePoints(cameraMatrix1, cameraMatrix2, point1Homogeneous, point2Homogeneous, point3D)
//
//		// Convert to 3D point
//		x := point3D.GetFloatAt(0, 0) / point3D.GetFloatAt(3, 0)
//		y := point3D.GetFloatAt(1, 0) / point3D.GetFloatAt(3, 0)
//		z := point3D.GetFloatAt(2, 0) / point3D.GetFloatAt(3, 0)
//
//		points3D = append(points3D, gocv.NewPoint3f(x, y, z))
//	}
//
//	return points3D, nil
//}

//func createMesh(points []stl.Point3D) *delaunay.Triangulation {
//	triangulation := delaunay.NewTriangulation()
//	for _, point := range points {
//		triangulation.Add(delaunay.Vertex{point.X, point.Y, point.Z})
//	}
//	triangulation.Build()
//
//	return triangulation
//}
//
//func generateSTLFromMesh(triangulation *delaunay.Triangulation, filename string) error {
//	mesh := stl.Mesh{}
//	for _, triangle := range triangulation.Triangles {
//		mesh.AddTriangle(
//			stl.Triangle{
//				stl.Coord(triangle[0].X, triangle[0].Y, triangle[0].Z),
//				stl.Coord(triangle[1].X, triangle[1].Y, triangle[1].Z),
//				stl.Coord(triangle[2].X, triangle[2].Y, triangle[2].Z),
//			},
//		)
//	}
//
//	// Create the output STL file
//	stlFile, err := os.Create(filename)
//	if err != nil {
//		return err
//	}
//	defer stlFile.Close()
//
//	// Create an STL encoder
//	encoder := stl.NewEncoder(stlFile)
//
//	// Write the mesh to the STL file
//	if err := encoder.Encode(mesh); err != nil {
//		return err
//	}
//
//	fmt.Printf("STL file '%s' created successfully.\n", filename)
//
//	return nil
//}

func TriangulatePoints(projMatr1, projMatr2, projPoints1, projPoints2 []float64, n int) []float64 {
	points4D := make([]float64, 4*n)
	C.TriangulatePoints(
		(*C.double)(&projMatr1[0]),
		(*C.double)(&projMatr2[0]),
		(*C.double)(&projPoints1[0]),
		(*C.double)(&projPoints2[0]),
		C.int(n),
		(*C.double)(&points4D[0]))
	return points4D
}

func extract2DPoints(matches []gocv.DMatch, keypoints1, keypoints2 []gocv.KeyPoint) (img1Points, img2Points []gocv.Point2f) {
	for _, match := range matches {
		point1 := new(gocv.Point2f)
		point1.X = float32(keypoints1[match.QueryIdx].X)
		point1.Y = float32(keypoints1[match.QueryIdx].Y)
		point2 := new(gocv.Point2f)
		point2.X = float32(keypoints2[match.TrainIdx].X)
		point2.Y = float32(keypoints2[match.TrainIdx].Y)

		img1Points = append(img1Points, *point1)
		img2Points = append(img2Points, *point2)
	}
	return img1Points, img2Points
}

func main() {
	// Read images from disk.
	images, err := readImagesFromDirectory("./data/test1")
	if err != nil {
		panic(err)
	}

	// Load images and perform feature matching to establish correspondences.
	matches, _, _, _, err := featureMatching(images)
	if err != nil {
		panic(err)
	}

	println(len(matches))
	spew.Dump(matches)

	//// Prepare camera matrices.
	//projMatrix1 := cameras[0].Ma // The projection matrix for the first camera.
	//projMatrix2 := cameras.ProjMatrix2 // The projection matrix for the second camera.
	//
	//for _, match := range matches {
	//	// Extract 2D point correspondences from keypoints and matches.
	//	img1Points, img2Points := extract2DPoints(match, keypoints1, keypoints2)
	//
	//	// Triangulate 3D points using the correspondences.
	//	points4D := TriangulatePoints(
	//		projMatrix1,
	//		projMatrix2,
	//		img1Points,      // Corresponding 2D points in image 1
	//		img2Points,      // Corresponding 2D points in image 2
	//		len(img1Points), // Number of points
	//	)
	//	if err != nil {
	//		panic(err)
	//	}
	//	spew.Dump(points4D)
	//}

	// Create a mesh from the 3D points.
	//mesh := createMesh(points)
	//err = generateSTLFromMesh(mesh, "./output/test1.stl")
	//if err != nil {
	//	panic(err)
	//}

	// Perform bundle adjustment to refine camera parameters and 3D points.

	// Visualize or save the 3D reconstruction.
}
