package clearance

import "math"

// TrigRange is the exact range of a·cosθ + b·sinθ over [lo, hi].
func TrigRange(a, b, lo, hi float64) (float64, float64) {
	mn := math.Min(a*math.Cos(lo)+b*math.Sin(lo), a*math.Cos(hi)+b*math.Sin(hi))
	mx := math.Max(a*math.Cos(lo)+b*math.Sin(lo), a*math.Cos(hi)+b*math.Sin(hi))
	if a == 0 && b == 0 {
		return 0, 0
	}
	star := math.Atan2(b, a)
	for _, cand := range []float64{star, star + math.Pi} {
		for kk := math.Floor((lo-cand)/(2*math.Pi)) * 2 * math.Pi; cand+kk <= hi+1e-12; kk += 2 * math.Pi {
			th := cand + kk
			if th < lo-1e-12 {
				continue
			}
			v := a*math.Cos(th) + b*math.Sin(th)
			mn = math.Min(mn, v)
			mx = math.Max(mx, v)
		}
	}
	return mn, mx
}
