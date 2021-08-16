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
	mat  gocv.Mat
	curr int64
	cost float32
	mean float64
}

// range of search & initial step
type Set_AF struct {
	start int64
	end   int64
	step  int64
	peak  int64
	prob  float32
	roi   i.Rectangle
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

	slope_step int64 = 500 // slope_step

	frame_cnt int   = 0 // frame counter
	dir       int64 = 0 // direction

	MAX int64 = 60000
	MIN int64 = 40500
)

// slope calculation
func calSlope(past float64, present float64, d int64) float64 {
	var (
		slp_x float64 = float64(d * slope_step)
		slp_y float64 = present - past
	)
	slp := slp_y / slp_x

	fmt.Println("dir : ", d)
	fmt.Println("slp_y : ", slp_y, "slp_x : ", slp_x, "slp : ", slp)

	return slp
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
	time.Sleep(20 * time.Millisecond)

	af_roiImg = GetImage()
	af_slope_info[frame_cnt] = GetContrastForAF(af_roiImg)
	fmt.Println("af_info[frame_cnt] : ", af_info[frame_cnt], "af_slope_info[frame_cnt] : ", af_slope_info[frame_cnt])
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

	af_state := af.Hill_climbing()

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
func (s *Set_AF) Hill_climbing() IP_STATE {
	// Initialize variables
	var (
		ip_state IP_STATE = IP_NOT_FOUND // state of image processing

		move_step int64 = s.step              // real len of step
		fst_step  int64 = s.step - slope_step // practical len of step considering slope

		past_slope float64 = 0 // past slope for determining moving direction
		curr_slope float64 = 0 // current slope for determining moving direction
	)

	start := time.Now() // start to count

	curr_pos := rtuc.Handle.Get_int64(rtuc.CONF_af_curr)

	frame_cnt = 0
	dir = -1
	//end_flag := 0
	turn_flag := 0
	cnt := 0

	var delay time.Duration = 500 // Motor - Image sync

	// Auto-Focus start!!
	for ip_state != IP_SUCCESS {

		if Ip_image.Empty() {
			fmt.Println("Ip_image is empty...")
			continue
		}

		// Initialize & first-time
		if frame_cnt == 0 {
			af_info = make([]__AF, 100)
			af_slope_info = make([]__AF, 100)

			fmt.Println("AF init...")

			//Move within the range
			if curr_pos <= MIN {
				motor.ActivateToPos(0, 0, 0, MIN)
			} else if curr_pos >= MAX {
				motor.ActivateToPos(0, 0, 0, MAX)
			}

			time.Sleep(delay * time.Millisecond)

			getPoint()

			//Calculate the slope of starting point
			past_slope = calSlope(af_info[frame_cnt].mean, af_slope_info[frame_cnt].mean, dir)
			fmt.Println("first_slope : ", past_slope)

			// Set first moving direction
			if past_slope > 0 {
				dir = 1
				motor.MoveAF(dir * move_step)
			} else {
				dir = -1
				motor.MoveAF(dir * fst_step)
			}

			time.Sleep(delay * time.Millisecond)

			frame_cnt++
			ip_state = IP_FINDING

			continue
		}

		// Hill-climbing AF
		getPoint()

		curr_slope = calSlope(af_info[frame_cnt].mean, af_slope_info[frame_cnt].mean, dir)

		fmt.Println("past_slope : ", past_slope, "curr_slope : ", curr_slope)

		// determining moving direction & moving step
		if (past_slope > 0 && curr_slope < 0) || (past_slope < 0 && curr_slope > 0) {
			dir = -dir
			move_step = move_step / 2
			delay = delay / 2

			turn_flag++
			cnt = 0
		}

		if turn_flag == 1 && cnt == 1 {
			move_step = move_step / 2
			delay = delay / 2

			motor.MoveAF(dir * (move_step - slope_step))
			ip_state = IP_SUCCESS
			break

		} else if turn_flag == 2 && cnt == 0 {
			motor.MoveAF(dir * (move_step + slope_step))
			ip_state = IP_SUCCESS
			break
			/*} else if turn_flag == 0 && (curr_pos+dir*move_step) > MAX {
				if end_flag > 0 {
					ip_state = IP_SUCCESS
					break
				}
				motor.ActivateToPos(0, 0, 0, MAX)
				end_flag++

			} else if turn_flag == 0 && (curr_pos+dir*move_step) < MIN {
				if end_flag > 0 {
					ip_state = IP_SUCCESS
					break
				}

				motor.ActivateToPos(0, 0, 0, MIN)
				end_flag++ */

		} else {
			motor.MoveAF(dir * move_step)
		}

		time.Sleep(delay * time.Millisecond)

		past_slope = curr_slope
		frame_cnt++
		cnt++
	}

	elapsed := time.Since(start)
	log.Printf("<----------Autofocus time took %s", elapsed)
	return ip_state
}

func (s *Set_AF) Auto_focus() IP_STATE {
	var (
		ip_state         IP_STATE = IP_NOT_FOUND
		af_frame_counter int      = 0
		move_flag        bool     = false
		direction        int64    = 0
		step_number      int64    = 0
	)
	start := time.Now()
	for ip_state != IP_SUCCESS {

		if Ip_image.Empty() {
			fmt.Println("Ip_image is empty...")
			continue
		}

		var (
			af_image gocv.Mat = gocv.NewMat()
			//resized  gocv.Mat = gocv.NewMat()
		)

		Ip_image.CopyTo(&af_image)
		//af_image := Ip_image.Clone()

		// focuszone := i.Rect(0, 0, 0, 0)
		// if s.roi.Min.X == 0 {
		// 	focuszone = i.Rect(100, 140, 540, 340)
		// } else {
		// 	focuszone = s.roi
		// }
		// //gocv.Resize(af_image, &resized, image.Pt(160, 120), 0, 0, gocv.InterpolationCubic)
		// //afzone := i.Rect(25, 35, 135, 85) //autofocus zone
		// af_roiImg := af_image.Region(focuszone)
		af_roiImg := af_image
		//set motor to initial position
		if af_frame_counter == 0 {
			//Initialize
			step_number = int64((s.start - s.end) / s.step)
			if step_number > 0 {
				direction = +1
				af_info = make([]__AF, (step_number+1)*direction)

			} else {
				direction = -1
				af_info = make([]__AF, (step_number-1)*direction)

			}
			motor.ActivateToPos(0, 0, 0, s.start)

			s.peak = 0
			af_frame_counter++
			ip_state = IP_FINDING
			fmt.Println("af init ..."+"step_number:", step_number)
			continue
		}

		if direction > 0 {
			if rtuc.Handle.Get_int64("af_curr") >= s.end {
				af_info[af_frame_counter-1] = GetContrastForAF(af_roiImg)

				move_flag = true
			} else {
				move_flag = false
			}
		} else {
			if rtuc.Handle.Get_int64("af_curr") <= s.end {
				af_info[af_frame_counter-1] = GetContrastForAF(af_roiImg)

				move_flag = true
			} else {
				move_flag = false
			}
		}

		if move_flag {
			af_frame_counter++
			motor.MoveAF(-s.step * direction)
			time.Sleep(20 * time.Millisecond)
			ip_state = IP_FOUND_AND_MOVE
		} else {
			i := 0
			var peak_mean float64 = 0
			for i < int(step_number) {
				curr_mean := af_info[i].mean
				curr_pos := af_info[i].curr
				if curr_mean > peak_mean {
					peak_mean = curr_mean
					s.peak = curr_pos
				}
				fmt.Println("#", i, "-> pos: ", curr_pos, " value:", curr_mean)
				i++
			}
			motor.ActivateToPos(0, 0, 0, s.peak*direction)
			ip_state = IP_SUCCESS
		}

	}
	elapsed := time.Since(start)
	log.Printf("<----------Autofocus time took %s", elapsed)
	return ip_state
}
