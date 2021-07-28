# Auto Focus Algorithm 🦄

---

- Background
- Hill-climbing Search
- Code Review
- Search Flow
- Performance
- Feedback

## Background 🐘

---

- **Global Search(Best accuracy)**

    Global search strategy goes through every possible position in unidirectional manner. This search guarantees to find optimal focus however, this is the slowest method.

- **Two-step Search(Present model)**

    Two-step search is a fast version of the Global search. This search is faster than Global but, less accurate.

- **Fibonacci Search(Second best performance)**

    Fibonacci search strategy is based on continuously narrowing the search region according to the Fibonacci sequence. The required number of iterations is also given by Fibonacci sequence.

- **Hill-climbing Search(Best performance)**

    Hill climbing search determines the direction of the next movement by the gradient of the previous two consecutive focus values. The movement step is determined by a parameter L. When the direction of the lens movement is reversed, L is reduced to one half of its original value for a finer search. (Although this search can start at any lens position, it it sensitive to image noise, and its performance depends on the chosen parameter values.)

Reference

[autofocus.pdf](Auto%20Focus%20Algorithm%20154b800e73214a2fac798579d3fc060f/autofocus.pdf)

## Hill-climbing Search 🐇

---

- **Find parameter value that minimizes motor steps**

    **#Local search #Gradient ascent/descent #Parameter L #Motor steps #Local minima/maxima**

- **Code**

    ```go
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

    		prac_step int64 = s.step - slope_step // practical len of step considering slope
    		move_step int64 = s.step              // real len of step
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
    			dir = -1
    			af_info = make([]__AF, 15)
    			af_slope_info = make([]__AF, 15)

    			motor.ActivateToPos(0, 0, 0, s.start)

    			getPoint()

    			past_slope = calSlope(af_info[frame_cnt].mean, af_slope_info[frame_cnt].mean)

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
    		if (past_slope > 0 && curr_slope < 0) || (past_slope < 0 && curr_slope > 0) {
    			dir = -dir
    			move_step = move_step / 2
    			prac_step = move_step - slope_step
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
    ```

## Code Review 🐩

---

```go
package image_processing

import (
	"fmt"
	i "image"
	"log"
	"time"

	motor "github.com/RooteeHealth/ALI/tree/ALI_Go/internal/motor_control"
	rtuc "github.com/RooteeHealth/ALI/tree/ALI_Go/use_case" // DB access
	"gocv.io/x/gocv"                                        // goCV
)

/***********************
autofocus functions
************************/

type __AF struct { // AF에 대한 value
	mat  gocv.Mat
	curr int64   // Current motor position
	cost float32 // Mid-point of strong edge
	mean float64 // Actual value
}

type Set_AF struct {
	start int64
	end   int64
	step  int64
	peak  int64
	prob  float32
	roi   i.Rectangle // Selected point
}

func (s *Set_AF) Set(start int64, end int64, step int64) { //
	s.start = start
	s.end = end
	s.step = step //

	if step != 0 {
		s.step = step
	} else {
		s.step = 200 // ?????
	}
}

/*
type FindAF struct {    // for Test
	cnt       int
	value     float64
	pos       int64
	slope_val float64
	slope     bool
}
*/

var (
	current __AF
	// AF_STEP int64 = 50
	// FindAF_Info []FindAF = make([]FindAF, 1000, 1000)

	af_info     []__AF
	af_sub_info []__AF
)

/*
func SimpleAutoFocus(unit_step int64) IP_STATE {  // 테스트용

	var (
		ip_state       IP_STATE
		unitstep       int64  = unit_step //starting value
		af_cnt         int    = 1
		move_direction int    = 1 // 1: clockwise -1:anti clowise
		PeakPos        FindAF = FindAF{0, 0, 0, 0, false}
		af_image       gocv.Mat
	)

	//move to af initial position --> af_base

	for ip_state != IP_SUCCESS {

		if Ip_image.Empty() {
			fmt.Println("Ip_image is empty...")
			return IP_FAIL
		}

		if af_cnt > 100 {
			af_cnt = 0
			FindAF_Info = nil
			motor.ActivateToPos(0, 0, 0, rtuc.Handle.Get_int64("af_max"))
		}

		var af FindAF
		unitstep = unitstep * int64(move_direction)

		FindAF_Info[0] = FindAF{0, 0, 0, 0, false}
		af_image = Ip_image.Clone()
		//gocv.Resize(Ip_image, &af_image, i.Pt(640, 480), 0, 0, gocv.InterpolationCubic)

		//af struct
		af.cnt = af_cnt
		af.pos = rtuc.Handle.Get_int64("af_curr")
		af.value = GetContrastForAF(af_image)
		af.slope_val = CalSlopePoints(af_cnt)
		if af.slope_val < 0 {
			af.slope = false
		} else {
			af.slope = true
		}

		//Move 5 steps
		motor.ActivateToPos(0, 0, 0, af.pos-unitstep)
		FindAF_Info[af_cnt] = af

		if af_cnt%5 == 0 {
			j := 5
			curve := 0
			var sum float64
			for j > 0 {
				sum = sum + FindAF_Info[af_cnt-j].slope_val
				if FindAF_Info[af_cnt-j].slope {
					curve++
				} else {
					curve--
				}

				if FindAF_Info[af_cnt-j].value > PeakPos.value {
					PeakPos.value = FindAF_Info[af_cnt-j].value
					PeakPos.pos = FindAF_Info[af_cnt-j].pos
					PeakPos.slope_val = FindAF_Info[af_cnt-j].slope_val
				}
				j--
			}
			mean := sum / 5
			fmt.Println("af_cnt:", af_cnt, "curve:", curve, "mean:", "peak_pos:", mean, PeakPos.pos, "peak_val:", PeakPos.value)

			if mean < 0 {
				//move to curved pos (minimum slope),
				if curve <= 1 && curve >= -1 {
					if FindAF_Info[af_cnt-4].pos < PeakPos.pos && FindAF_Info[af_cnt].pos > PeakPos.pos {
						motor.ActivateToPos(0, 0, 0, PeakPos.pos)
						ip_state = IP_SUCCESS
					}
				}
				//if not exist, find biggest value's pos.
				if curve <= -3 {
					//move back to
					move_direction = -1
					motor.ActivateToPos(0, 0, 0, PeakPos.pos)
					ip_state = IP_FINDING
				}
			} else {
				move_direction = +1
				ip_state = IP_FINDING
			}
		}
		af_cnt++
	}
	return IP_EMPTY
}
*/

func CalSlopePoints(cnt int) float64 { // Calculation of Slope
	if cnt-1 < 0 {
		return 0
	}

	var p2 [2]float64 = [2]float64{FindAF_Info[cnt].value, float64(FindAF_Info[cnt].pos)}
	var p1 [2]float64 = [2]float64{FindAF_Info[cnt-1].value, float64(FindAF_Info[cnt-1].pos)}

	return float64((p2[1] - p1[1]) / (p2[0] - p1[0]))
}

func calSlope(past int64, present int64) float64 { //
	return float64((present - past) / Dx)
}

func GetContrastForAF(img gocv.Mat) float64 { // Get Contrast value
	sobel_out := get_sobel_image(img)
	current.curr = rtuc.Handle.Get_int64(rtuc.CONF_af_curr) // DB에 저장 일단 데이터 수집 후 비교
	current.mean = sobel_out.Mean().Val1 * 100.0            // Get contrast

	return current
}

func GetImage() gocv.Mat {
	var temp gocv.Mat = gocv.NewMat()
	Ip_image.CopyTo(&temp)
	return temp
}

/*
func SampleAutofocus() IP_STATE {  // for Test
	var (
		af1      Set_AF
		af2      Set_AF
		ip_state IP_STATE
	)
	af1.Set(
		rtuc.Handle.Get_int64("af_base"),
		rtuc.Handle.Get_int64("af_base")-4000,
		1000,
	)
	af1_state := af1.Auto_focus()
	if af1_state == IP_SUCCESS {
		af2.Set(
			af1.peak+300,
			af1.peak-300,
			100,
		)
		af2_state := af2.Auto_focus()

		if af2_state == IP_SUCCESS {
			ip_state = IP_SUCCESS
			fmt.Println("final peak:", af2.peak)
		}
	}
	return ip_state
}
*/

// First Search : Pupil Auto-Focus
func PupilAutoFocus(scan int64, step int64) IP_STATE {
	var (
		af Set_AF
		//ip_state IP_STATE
	)

	//Set ROI
	af.roi = Last_Roi // 이거 전역 변수네? -> 하르가 찾아주는듯

	//Set value
	af.Set(
		rtuc.Handle.Get_int64("af_base"),      // start    configs/db/db.yaml에서 af_base값 읽어오기
		rtuc.Handle.Get_int64("af_base")-scan, // end      시작점 65000 -> 65000-scan 까지
		step,                                  // step
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

	af1.Set(
		rtuc.Handle.Get_int64("af_curr"),
		rtuc.Handle.Get_int64("af_curr")-scan,
		step,
	)

	af1_state := af1.Auto_focus()

	return af1_state
}

// Two-steps Search Algorithm
func (s *Set_AF) Auto_focus() IP_STATE {
	var (
		ip_state         IP_STATE = IP_NOT_FOUND // image processing 완료 여부
		af_frame_counter int      = 0
		move_flag        bool     = false
		direction        int64    = 0
		step_number      int64    = 0
		slope_step       int      = 312 //
	)
	start := time.Now()          // 시작시간
	for ip_state != IP_SUCCESS { // 이미지 프로세싱 완료 안되었다면 한번 더 고고

		if Ip_image.Empty() {
			fmt.Println("Ip_image is empty...")
			continue
		}

		af_roiImg := GetImage() // 하르로 pupil detecting해서 입력받는건가?

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

		//
		if af_frame_counter == 0 {
			direction = 1
			af_info = make([]__AF, 15)
			af_sub_info = make([]__AF, 15)

			motor.ActivateToPos(0, 0, 0, s.start)

			af_info[af_frame_counter] = GetContrastForAF(af_roiImg)

			motor.MoveAF(-slope_step)

			af_roiImg := GetImage()

			af_sub_info[af_frame_counter] = GetContrastForAF(af_roiImg)
		}

		//set motor to initial position
		if af_frame_counter == 0 { // 처음이면
			//Initialize
			step_number = int64((s.start - s.end) / s.step) // distance / step -> step_number, step 넘버 구하고
			if step_number > 0 {                            // 보통 끝을 start로 잡고 end를 0으로
				direction = +1
				af_info = make([]__AF, (step_number+1)*direction)
			} else {
				direction = -1
				af_info = make([]__AF, (step_number-1)*direction)
			}
			motor.ActivateToPos(0, 0, 0, s.start) // 시작 지점으로 옮기고

			s.peak = 0
			af_frame_counter++
			ip_state = IP_FINDING // finding flag
			fmt.Println("af init ..."+"step_number:", step_number)
			continue
		}

		// get contrast value
		if direction > 0 {
			if rtuc.Handle.Get_int64("af_curr") >= s.end {
				sobel_out := get_sobel_image(af_roiImg)                 // Filter : sobel_filter
				current.curr = rtuc.Handle.Get_int64(rtuc.CONF_af_curr) // DB에 저장 일단 데이터 수집 후 비교
				current.mean = sobel_out.Mean().Val1 * 100.0            // Get contrast
				af_info[af_frame_counter-1] = current                   // array에 contrast 저장
				move_flag = true
			} else {
				move_flag = false
			}
		} else {
			if rtuc.Handle.Get_int64("af_curr") <= s.end {
				sobel_out := get_sobel_image(af_roiImg) // Filter : Strong Edge
				current.curr = rtuc.Handle.Get_int64(rtuc.CONF_af_curr)
				current.mean = sobel_out.Mean().Val1 * 100.0 // Get contrast
				af_info[af_frame_counter-1] = current
				move_flag = true
			} else {
				move_flag = false
			}
		}

		// Move to Flag = false, 끝에서 처음으로 이동하는 구조
		if move_flag { // 다음으로 이동하고
			af_frame_counter++
			motor.MoveAF(-s.step * direction)
			time.Sleep(20 * time.Millisecond)
			ip_state = IP_FOUND_AND_MOVE
		} else { // 전 범위 search 끝내고 최고값 찾아서 이동하고 끝
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
			motor.ActivateToPos(0, 0, 0, s.peak*direction) // move to optimal contrast
			ip_state = IP_SUCCESS
		}
	}
	elapsed := time.Since(start)                             // 종료시간
	log.Printf("<----------Autofocus time took %s", elapsed) // af
	return ip_state
}
```

## Search Flow 🦛

---

- **Overall Flow**

     `Start` 

        ⬇️

    `Initialize`

        ⬇️

    `Stream`

        ⬇️

    `Focusing Region Detection (get X, Y coordinate)`

        ⬇️

    `Pupil Auto-focus(two-steps search)`

        ⬇️

    `Move Z (adjust distance)`

        ⬇️

    `Retina Auto-focus(two-steps search)`

        ⬇️

     `Shot`  

        ⬇️

    `Finish`

- **Auto Focus Flow**

       `Camera`            ➡️            `Image Frame Capture` 

                                                             ⬇️                      

                                          `Focusing Region Selection` 

                                                             ⬇️

           ⬆️                                       `Strong Edge`

                                                             ⬇️ 

                                              `Contrast Measurement` 

                                                             ⬇️

`Adjust AF-axis`        ⬅️          `Search Algorithm`

                                                             ⬇️

                                             `Optimal Focused Image`

- **Hill-climbing Search Flow**

`Set parameter`

⬇️

`Get image input`

⬇️

`Get contrast`

⬇️

`Calculate slope at the current point`  

⬇️

`if slope ≥ 0`      ➡️  no      `Global search`

⬇️  yes

`Move +L`

⬇️

`Calculate slope at the next point`    ⬅️⬅️⬅️⬅️⬅️⬅️⬅️

⬇️                                                                                     ⬆️

`If next slope ≥ 0`   ➡️  no   `Move -1/2 * L` ➡️➡️➡️  ⬆️

⬇️  yes                                                                             ⬆️

`Move +L`   ➡️➡️➡️➡️➡️➡️➡️➡️➡️➡️➡️➡️➡️➡️➡️➡️ ⬆️

⬇️

`If optimal`

⬇️

`Done`

## Performance 🕊️

---

[Pupil](https://www.notion.so/8fd99309cf194256aa46ab9cdfa5f519)

[Retina](https://www.notion.so/ee2544272f9c4b48a265f139b9c76309)

*Think of Geometric Series. If the geometric ratio is less than 1, then the geometric series must be less than 1. As a result, We can get profits of distance by changing search algorithm.

## Feedback 🦈

---

- Experimental initial point
- Optimal parameter value
- Detailed comments
- Refactory & Clean code
- We need to reduce delay_time

- reference → share my research
- bad contrast algorithm → need efficiency
- search twice at one point

---

by The Legendary Dragon 🐉