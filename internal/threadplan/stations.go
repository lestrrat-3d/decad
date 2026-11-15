package threadplan

import "math"

// StationsPerTurn chooses a reduced helix sampling density when the groove's
// radial steps and chord departure fit the held-shell limits. It returns
// maxStations when neither reduced density fits.
func StationsPerTurn(plan Plan, pitch float64, maxStations int64) int64 {
	const reducedStations = 96
	const priorReducedStations = 128
	const maxEstimatedDeparture = 0.003

	rhoMin, rhoMax := math.Inf(1), math.Inf(-1)
	maxRadialStep := 0.0
	for i, point := range plan.Profile {
		rhoMin = min(rhoMin, point[0])
		rhoMax = max(rhoMax, point[0])
		if i > 0 {
			maxRadialStep = max(maxRadialStep, math.Abs(point[0]-plan.Profile[i-1][0]))
		}
	}
	maxRadialStep = max(maxRadialStep, math.Abs(plan.Profile[0][0]-plan.Profile[len(plan.Profile)-1][0]))
	if !(rhoMin > 0) || maxRadialStep > rhoMin {
		return maxStations
	}
	allowedDeparture := min(maxEstimatedDeparture, 0.0006*rhoMax)
	k := pitch / (2 * math.Pi)
	departure := func(stations int64) float64 {
		h := math.Pi / float64(stations)
		return rhoMax*h*h/2 + maxRadialStep*h/2*(h+k/rhoMin) +
			maxRadialStep*maxRadialStep*h*h/(8*rhoMin)
	}
	if reducedStations < maxStations && departure(reducedStations) <= allowedDeparture {
		return reducedStations
	}
	if priorReducedStations < maxStations && departure(priorReducedStations) <= allowedDeparture {
		return priorReducedStations
	}
	return maxStations
}
