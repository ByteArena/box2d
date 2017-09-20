package box2d

func MinInt(x, y int) int {
	if x < y {
		return x
	}
	return y
}

func MaxInt(x, y int) int {
	if x > y {
		return x
	}
	return y
}

func AbsInt(v int) int {
	if v < 0 {
		return v * -1
	}

	return v
}
