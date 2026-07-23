package decad

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"strings"
)

// recipeDecodeLimits are the fixed wire-admission limits used by Recipe's
// json.Unmarshaler. The legacy JSON entry point has no option channel.
type recipeDecodeLimits struct {
	MaxBytes            int64
	MaxDepth            int
	MaxSteps            int
	MaxLoops            int
	MaxSegments         int
	MaxCurvePoints      int
	MaxCurveScalars     int
	MaxSelectors        int
	MaxPredicates       int
	MaxRoleBytes        int
	MaxTotalStringBytes int
}

func defaultRecipeDecodeLimits() recipeDecodeLimits {
	return recipeDecodeLimits{
		MaxBytes:            16 << 20,
		MaxDepth:            32,
		MaxSteps:            4_096,
		MaxLoops:            65_536,
		MaxSegments:         262_144,
		MaxCurvePoints:      1_048_576,
		MaxCurveScalars:     1_048_576,
		MaxSelectors:        16_384,
		MaxPredicates:       131_072,
		MaxRoleBytes:        256,
		MaxTotalStringBytes: 1 << 20,
	}
}

// UnmarshalJSON admits the complete wire value before encoding/json can grow
// Recipe's typed slices. Decoding into a temporary also leaves the receiver
// unchanged when either admission or typed decoding fails.
func (r *Recipe) UnmarshalJSON(data []byte) error {
	if err := preflightRecipeJSON(data, defaultRecipeDecodeLimits()); err != nil {
		return err
	}
	type recipeWire Recipe
	var out recipeWire
	if err := json.Unmarshal(data, &out); err != nil {
		return err
	}
	*r = Recipe(out)
	return nil
}

type recipeJSONRole uint8

const (
	recipeJSONIgnore recipeJSONRole = iota
	recipeJSONRoot
	recipeJSONSteps
	recipeJSONStep
	recipeJSONProfile
	recipeJSONOuterLoop
	recipeJSONLoops
	recipeJSONLoop
	recipeJSONSegments
	recipeJSONSegment
	recipeJSONCurvePoints
	recipeJSONCurveScalars
	recipeJSONSelectors
	recipeJSONSelectorValue
	recipeJSONSelector
	recipeJSONPredicates
	recipeJSONPredicate
	recipeJSONFeatureRef
	recipeJSONLinearExtent
	recipeJSONSideExtent
	recipeJSONAngularExtent
	recipeJSONSideAngular
	recipeJSONAxis
	recipeJSONOpts
	recipeJSONValues
	recipeJSONString
	recipeJSONRoleString
)

type recipeJSONLimit uint8

const (
	recipeLimitSteps recipeJSONLimit = iota
	recipeLimitLoops
	recipeLimitSegments
	recipeLimitCurvePoints
	recipeLimitCurveScalars
	recipeLimitSelectors
	recipeLimitPredicates
)

type recipeJSONPreflight struct {
	decoder *json.Decoder
	limits  recipeDecodeLimits
	depth   int

	steps           int
	loops           int
	segments        int
	curvePoints     int
	curveScalars    int
	selectors       int
	predicates      int
	totalStringByte int
}

// preflightRecipeJSON scans without retaining aggregate values. Array
// elements are charged before descent, so the later typed decoder never sees
// an admitted recipe whose schema slices exceed their fixed ceilings.
func preflightRecipeJSON(data []byte, limits recipeDecodeLimits) error {
	if int64(len(data)) > limits.MaxBytes {
		return recipeDecodeLimitError("MaxBytes", limits.MaxBytes)
	}

	decoder := json.NewDecoder(bytes.NewReader(data))
	decoder.UseNumber()
	scan := recipeJSONPreflight{decoder: decoder, limits: limits}
	if err := scan.value(recipeJSONRoot); err != nil {
		return err
	}
	if _, err := decoder.Token(); err != io.EOF {
		if err == nil {
			return fmt.Errorf("decad: recipe JSON contains trailing data")
		}
		return fmt.Errorf("decad: failed to scan recipe JSON: %w", err)
	}
	return nil
}

func (s *recipeJSONPreflight) value(role recipeJSONRole) error {
	switch role {
	case recipeJSONOuterLoop:
		if err := s.charge(recipeLimitLoops); err != nil {
			return err
		}
		role = recipeJSONLoop
	case recipeJSONSelectorValue:
		if err := s.charge(recipeLimitSelectors); err != nil {
			return err
		}
		role = recipeJSONSelector
	}

	token, err := s.decoder.Token()
	if err != nil {
		return fmt.Errorf("decad: failed to scan recipe JSON: %w", err)
	}
	if delim, ok := token.(json.Delim); ok {
		if err := s.enterContainer(); err != nil {
			return err
		}
		defer func() { s.depth-- }()

		switch delim {
		case '{':
			return s.object(role)
		case '[':
			return s.array(role)
		default:
			return fmt.Errorf("decad: unexpected recipe JSON delimiter %q", delim)
		}
	}

	text, ok := token.(string)
	if !ok {
		return nil
	}
	switch role {
	case recipeJSONRoleString:
		if len(text) > s.limits.MaxRoleBytes {
			return recipeDecodeLimitError("MaxRoleBytes", int64(s.limits.MaxRoleBytes))
		}
		return s.chargeString(len(text))
	case recipeJSONString:
		return s.chargeString(len(text))
	default:
		return nil
	}
}

func (s *recipeJSONPreflight) enterContainer() error {
	if s.depth >= s.limits.MaxDepth {
		return recipeDecodeLimitError("MaxDepth", int64(s.limits.MaxDepth))
	}
	s.depth++
	return nil
}

func (s *recipeJSONPreflight) object(role recipeJSONRole) error {
	for s.decoder.More() {
		token, err := s.decoder.Token()
		if err != nil {
			return fmt.Errorf("decad: failed to scan recipe JSON object: %w", err)
		}
		key, ok := token.(string)
		if !ok {
			return fmt.Errorf("decad: recipe JSON object key is not a string")
		}
		if err := s.value(recipeJSONFieldRole(role, key)); err != nil {
			return err
		}
	}
	token, err := s.decoder.Token()
	if err != nil {
		return fmt.Errorf("decad: failed to close recipe JSON object: %w", err)
	}
	if token != json.Delim('}') {
		return fmt.Errorf("decad: unexpected recipe JSON object delimiter %q", token)
	}
	return nil
}

func (s *recipeJSONPreflight) array(role recipeJSONRole) error {
	elementRole, limit, metered := recipeJSONArrayElement(role)
	for s.decoder.More() {
		if metered {
			if err := s.charge(limit); err != nil {
				return err
			}
		}
		if err := s.value(elementRole); err != nil {
			return err
		}
	}
	token, err := s.decoder.Token()
	if err != nil {
		return fmt.Errorf("decad: failed to close recipe JSON array: %w", err)
	}
	if token != json.Delim(']') {
		return fmt.Errorf("decad: unexpected recipe JSON array delimiter %q", token)
	}
	return nil
}

func (s *recipeJSONPreflight) charge(limit recipeJSONLimit) error {
	var count *int
	var maximum int
	var name string
	switch limit {
	case recipeLimitSteps:
		count, maximum, name = &s.steps, s.limits.MaxSteps, "MaxSteps"
	case recipeLimitLoops:
		count, maximum, name = &s.loops, s.limits.MaxLoops, "MaxLoops"
	case recipeLimitSegments:
		count, maximum, name = &s.segments, s.limits.MaxSegments, "MaxSegments"
	case recipeLimitCurvePoints:
		count, maximum, name = &s.curvePoints, s.limits.MaxCurvePoints, "MaxCurvePoints"
	case recipeLimitCurveScalars:
		count, maximum, name = &s.curveScalars, s.limits.MaxCurveScalars, "MaxCurveScalars"
	case recipeLimitSelectors:
		count, maximum, name = &s.selectors, s.limits.MaxSelectors, "MaxSelectors"
	case recipeLimitPredicates:
		count, maximum, name = &s.predicates, s.limits.MaxPredicates, "MaxPredicates"
	default:
		panic("decad: unknown recipe JSON limit")
	}
	if *count >= maximum {
		return recipeDecodeLimitError(name, int64(maximum))
	}
	*count += 1
	return nil
}

func (s *recipeJSONPreflight) chargeString(n int) error {
	if n > s.limits.MaxTotalStringBytes-s.totalStringByte {
		return recipeDecodeLimitError("MaxTotalStringBytes", int64(s.limits.MaxTotalStringBytes))
	}
	s.totalStringByte += n
	return nil
}

func recipeDecodeLimitError(name string, maximum int64) error {
	return fmt.Errorf("decad: recipe decode exceeds %s (%d): %w", name, maximum, ErrResourceLimit)
}

func recipeJSONArrayElement(role recipeJSONRole) (recipeJSONRole, recipeJSONLimit, bool) {
	switch role {
	case recipeJSONSteps:
		return recipeJSONStep, recipeLimitSteps, true
	case recipeJSONLoops:
		return recipeJSONLoop, recipeLimitLoops, true
	case recipeJSONSegments:
		return recipeJSONSegment, recipeLimitSegments, true
	case recipeJSONCurvePoints:
		return recipeJSONIgnore, recipeLimitCurvePoints, true
	case recipeJSONCurveScalars:
		return recipeJSONIgnore, recipeLimitCurveScalars, true
	case recipeJSONSelectors:
		return recipeJSONSelector, recipeLimitSelectors, true
	case recipeJSONPredicates:
		return recipeJSONPredicate, recipeLimitPredicates, true
	case recipeJSONValues:
		return recipeJSONString, 0, false
	default:
		return recipeJSONIgnore, 0, false
	}
}

func recipeJSONFieldRole(parent recipeJSONRole, field string) recipeJSONRole {
	switch parent {
	case recipeJSONRoot:
		if strings.EqualFold(field, "steps") {
			return recipeJSONSteps
		}
	case recipeJSONStep:
		switch {
		case strings.EqualFold(field, "op"):
			return recipeJSONString
		case strings.EqualFold(field, "profile"):
			return recipeJSONProfile
		case strings.EqualFold(field, "extent"):
			return recipeJSONLinearExtent
		case strings.EqualFold(field, "angular"):
			return recipeJSONAngularExtent
		case strings.EqualFold(field, "axis"):
			return recipeJSONAxis
		case strings.EqualFold(field, "selectors"):
			return recipeJSONSelectors
		case strings.EqualFold(field, "opts"):
			return recipeJSONOpts
		case strings.EqualFold(field, "values"):
			return recipeJSONValues
		}
	case recipeJSONProfile:
		switch {
		case strings.EqualFold(field, "outer"):
			return recipeJSONOuterLoop
		case strings.EqualFold(field, "holes"):
			return recipeJSONLoops
		}
	case recipeJSONLoop:
		if strings.EqualFold(field, "segments") {
			return recipeJSONSegments
		}
	case recipeJSONSegment:
		switch {
		case strings.EqualFold(field, "control"), strings.EqualFold(field, "fit"):
			return recipeJSONCurvePoints
		case strings.EqualFold(field, "knots"), strings.EqualFold(field, "weights"):
			return recipeJSONCurveScalars
		case recipeJSONSegmentStringField(field):
			return recipeJSONString
		}
	case recipeJSONSelector:
		switch {
		case strings.EqualFold(field, "kind"):
			return recipeJSONString
		case strings.EqualFold(field, "preds"):
			return recipeJSONPredicates
		}
	case recipeJSONPredicate:
		switch {
		case strings.EqualFold(field, "kind"), strings.EqualFold(field, "l"):
			return recipeJSONString
		case strings.EqualFold(field, "ref"):
			return recipeJSONFeatureRef
		}
	case recipeJSONFeatureRef:
		if strings.EqualFold(field, "role") {
			return recipeJSONRoleString
		}
	case recipeJSONLinearExtent:
		switch {
		case recipeJSONLinearExtentStringField(field):
			return recipeJSONString
		case strings.EqualFold(field, "one"), strings.EqualFold(field, "two"):
			return recipeJSONSideExtent
		case strings.EqualFold(field, "face"):
			return recipeJSONSelectorValue
		}
	case recipeJSONSideExtent:
		switch {
		case recipeJSONLinearExtentStringField(field):
			return recipeJSONString
		case strings.EqualFold(field, "face"):
			return recipeJSONSelectorValue
		}
	case recipeJSONAngularExtent:
		switch {
		case recipeJSONAngularExtentStringField(field):
			return recipeJSONString
		case strings.EqualFold(field, "one"), strings.EqualFold(field, "two"):
			return recipeJSONSideAngular
		case strings.EqualFold(field, "face"):
			return recipeJSONSelectorValue
		}
	case recipeJSONSideAngular:
		switch {
		case recipeJSONAngularExtentStringField(field):
			return recipeJSONString
		case strings.EqualFold(field, "face"):
			return recipeJSONSelectorValue
		}
	case recipeJSONAxis:
		switch {
		case strings.EqualFold(field, "kind"):
			return recipeJSONString
		case strings.EqualFold(field, "edge"):
			return recipeJSONSelectorValue
		}
	case recipeJSONOpts:
		switch {
		case strings.EqualFold(field, "kind"), strings.EqualFold(field, "taper"), strings.EqualFold(field, "sense"):
			return recipeJSONString
		}
	}
	return recipeJSONIgnore
}

func recipeJSONSegmentStringField(field string) bool {
	return strings.EqualFold(field, "kind") ||
		strings.EqualFold(field, "radius") ||
		strings.EqualFold(field, "rx") ||
		strings.EqualFold(field, "ry") ||
		strings.EqualFold(field, "rotation")
}

func recipeJSONLinearExtentStringField(field string) bool {
	return strings.EqualFold(field, "kind") ||
		strings.EqualFold(field, "d") ||
		strings.EqualFold(field, "dir") ||
		strings.EqualFold(field, "offset")
}

func recipeJSONAngularExtentStringField(field string) bool {
	return strings.EqualFold(field, "kind") ||
		strings.EqualFold(field, "a") ||
		strings.EqualFold(field, "dir")
}
