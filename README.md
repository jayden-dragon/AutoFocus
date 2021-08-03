# Auto Focus Algorithm 🦄

---
 
- Background
- Hill-climbing Search
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

- **Direction& Distance**
	
	<img src="./direction_distance.png">

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

<img src="./table.png">


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