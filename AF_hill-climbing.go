////////////////////////////
/// Hill-Climbing Search ///
////////////////////////////

package image_processing

import (
	"fmt"
	i "image"
	"log"
	"time"

	motor "github.com/RooteeHealth/ALI/tree/ALI_Go/internal/motor_control" // motor control API
	rtuc "github.com/RooteeHealth/ALI/tree/ALI_Go/use_case"                // DB access API
	"gocv.io/x/gocv"                                                       // goCV
)

// AF input & output data structure
type __AF struct {
	mat  gocv.Mat // Preprocessed image matrix
	curr int64    // Current motor position
	mean float64  // Actual contrast value
}

// range of search & initial step
type Set_AF struct {
	start int64
	end   int64
	step  int64
}

// Set Set_AF
func (s *Set_AF) Set(start int64, end int64, step int64) {
	s.start = start
	s.end = end
	s.step = step
}

var (
	current __AF // current image data

	af_info       []__AF // accumulated image data
	af_slope_info []__AF // accumulated image data for slope

	slope_step int64 = 312 // slope_step

	frame_cnt int   = 0 // frame counter
	dir       int64 = 0 // direction

	past_slope float64 = 0 // past slope for determining moving direction
	curr_slope float64 = 0 // current slope for determining moving direction
)

// slope calculation
func calSlope(past float64, present float64) float64 {
	return (present - past) / float64(slope_step)
}

// Get new image
func GetImage() gocv.Mat {
	var temp gocv.Mat = gocv.NewMat()
	Ip_image.CopyTo(&temp)

	return temp
}

// Get Contrast value
func GetContrastForAF(img gocv.Mat) __AF {
	sobel_out := get_sobel_image(img)                       // sobel filter
	current.curr = rtuc.Handle.Get_int64(rtuc.CONF_af_curr) // access DB
	current.mean = sobel_out.Mean().Val1 * 100.0            // convert to contrast value

	return current
}

// Get af_info for slope calculation
func getPoint() {
	af_roiImg := GetImage()
	af_info[frame_cnt] = GetContrastForAF(af_roiImg)

	motor.MoveAF(dir * slope_step)
	time.Sleep(10 * time.Millisecond)

	af_roiImg = GetImage()
	af_slope_info[frame_cnt] = GetContrastForAF(af_roiImg)
}

// First Search : Pupil Auto-Focus
func PupilAutoFocus(scan int64, step int64) IP_STATE {
	var (
		af Set_AF
		//ip_state IP_STATE
	)

	//Set value
	af.Set( // reverse direction
		rtuc.Handle.Get_int64("af_base"),
		rtuc.Handle.Get_int64("af_base")-scan,
		step,
	)

	af_state := af.Auto_focus()

	//Reset
	Last_Roi = i.Rect(0, 0, 0, 0)

	return af_state
}

// Second Search : Retina Auto-Focus
func RetinaAutoFocus(scan int64, step int64) IP_STATE {
	var (
		af1 Set_AF
		//ip_state IP_STATE
	)

	af1.Set( // reverse direction
		rtuc.Handle.Get_int64("af_curr"),
		rtuc.Handle.Get_int64("af_curr")-scan,
		step,
	)

	af1_state := af1.Auto_focus()

	return af1_state
}

// Auto-Focus main
func (s *Set_AF) Auto_focus() IP_STATE {
	var ( // initialize variables
		ip_state IP_STATE = IP_NOT_FOUND // state of image processing

		dir int = -1

		move_step int64 = s.step              // real len of step
		prac_step int64 = s.step - slope_step // practical len of step considering slope
	)

	start := time.Now() // start to count

	// Auto-Focus start!!
	for ip_state != IP_SUCCESS {

		if Ip_image.Empty() {
			fmt.Println("Ip_image is empty...")
			continue
		}

		// Initialize & first-time
		if frame_cnt == 0 {
			af_info = make([]__AF, 15)
			af_slope_info = make([]__AF, 15)

			motor.ActivateToPos(0, 0, 0, s.start)

			if s.start < 400 {
				dir = 1
			} else {
				dir = -1
			}

			getPoint()

			past_slope = calSlope(af_info[frame_cnt].mean, af_slope_info[frame_cnt].mean)

			if past_slope > 0 {
				dir = 1
			} else {
				dir = -1
			}

			motor.MoveAF(dir * (prac_step - slope_step))
			time.Sleep(20 * time.Millisecond)

			frame_cnt++
			ip_state = IP_FINDING
			fmt.Println("AF init...")

			continue
		}

		// Hill-climbing AF
		getPoint()

		curr_slope = calSlope(af_info[frame_cnt].mean, af_slope_info[frame_cnt].mean)

		// determining moving direction & moving step
		if past_slope > 0 && curr_slope > 0 {
			dir = +1
		} else if past_slope > 0 && curr_slope < 0 {
			dir = -1
			move_step = move_step / 2
			prac_step = move_step - slope_step
		} else if past_slope < 0 && curr_slope > 0 {
			dir = +1
			move_step = move_step / 2
			prac_step = move_step - slope_step
		} else if past_slope < 0 && curr_slope < 0 {
			dir = -1
		}

		if (move_step / 2) < slope_step { // determining termination
			motor.MoveAF(dir * move_step)
			ip_state = IP_SUCCESS
			break
		} else { // move to next step
			motor.MoveAF(dir * prac_step)
		}

		past_slope = curr_slope
		frame_cnt++
		time.Sleep(20 * time.Millisecond)
	}

	elapsed := time.Since(start)
	log.Printf("<----------Autofocus time took %s", elapsed)
	return ip_state
}
