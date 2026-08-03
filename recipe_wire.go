package decad

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"reflect"
	"strings"
)

const recipeWireFormat = "decad.recipe"

// recipeWireLegacyVersion is the pre-loft wire vocabulary (replay design
// §2.1): the version an unversioned legacy envelope implies, and the lowest
// version a versioned envelope may declare. recipeWireVersion is the
// canonical version the encoder always writes and the highest version this
// decoder accepts; it adds OpLoft and LoftOpts.profile2/plane2. A version-1
// envelope carrying a "loft" step is invalid under the version-1 grammar and
// is never reinterpreted with version-2 rules.
const recipeWireLegacyVersion = 1
const recipeWireVersion = 2

// RecipeError reports a stored-recipe failure with its location and
// branchable identity. StepIndex is -1 for an envelope or root failure.
type RecipeError struct {
	StepIndex int
	Path      string
	Kind      error
	Err       error
}

// Error reports the recipe location and failure.
func (e *RecipeError) Error() string {
	if e == nil {
		return "<nil>"
	}
	if e.Err == nil {
		return fmt.Sprintf("%v at %s", e.Kind, e.Path)
	}
	return fmt.Sprintf("%v at %s: %v", e.Kind, e.Path, e.Err)
}

// Unwrap preserves the root recipe error identity.
func (e *RecipeError) Unwrap() error {
	if e == nil {
		return nil
	}
	return e.Kind
}

// Is matches both the root recipe identity and a specific wrapped cause.
func (e *RecipeError) Is(target error) bool {
	if e == nil {
		return false
	}
	return errors.Is(e.Kind, target) || errors.Is(e.Err, target)
}

type recipeWireEnvelope struct {
	Format  string `json:"format"`
	Version int    `json:"version"`
	Steps   []Step `json:"steps"`
}

type rawRecipeWire struct {
	Format  json.RawMessage `json:"format"`
	Version json.RawMessage `json:"version"`
	Steps   json.RawMessage `json:"steps"`
}

// MarshalJSON writes the canonical version-2 recipe envelope (replay design
// §2.1: the encoder always writes the current version). Nil and empty step
// slices both encode as an array.
func (r Recipe) MarshalJSON() ([]byte, error) {
	steps := r.Steps
	if steps == nil {
		steps = []Step{}
	}
	data, err := json.Marshal(recipeWireEnvelope{
		Format:  recipeWireFormat,
		Version: recipeWireVersion,
		Steps:   steps,
	})
	if err != nil {
		return nil, rootRecipeError(ErrInvalidRecipe, fmt.Errorf("failed to encode recipe: %w", err))
	}
	return data, nil
}

// UnmarshalJSON accepts the canonical version 1 and version 2 envelopes and
// the legacy unversioned {"steps": ...} form, which decodes under the
// version-1 grammar. It rejects unknown root fields and duplicate keys
// before typed step decoding, and rejects a "loft" step under version-1
// rules (replay design §2.1).
func (r *Recipe) UnmarshalJSON(data []byte) error {
	if err := preflightRecipeJSON(data, defaultRecipeDecodeLimits()); err != nil {
		return err
	}
	if err := validateRecipeJSONStructure(data); err != nil {
		return rootRecipeError(ErrInvalidRecipe, err)
	}

	var raw rawRecipeWire
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.DisallowUnknownFields()
	if err := dec.Decode(&raw); err != nil {
		return rootRecipeError(ErrInvalidRecipe, fmt.Errorf("failed to decode recipe envelope: %w", err))
	}

	formatPresent := raw.Format != nil
	versionPresent := raw.Version != nil
	stepsPresent := raw.Steps != nil
	versioned := formatPresent || versionPresent

	if !stepsPresent {
		return rootRecipeError(ErrInvalidRecipe, errors.New(`recipe is missing required field "steps"`))
	}
	// version defaults to the legacy grammar for an unversioned envelope
	// (replay design §2.1); a versioned envelope overrides it below.
	version := recipeWireLegacyVersion
	if versioned {
		if !formatPresent {
			return rootRecipeError(ErrInvalidRecipe, errors.New(`recipe is missing required field "format"`))
		}
		if !versionPresent {
			return rootRecipeError(ErrInvalidRecipe, errors.New(`recipe is missing required field "version"`))
		}

		var format string
		if err := json.Unmarshal(raw.Format, &format); err != nil {
			return rootRecipeError(ErrInvalidRecipe, fmt.Errorf(`invalid recipe field "format": %w`, err))
		}
		if format != recipeWireFormat {
			return rootRecipeError(ErrInvalidRecipe, fmt.Errorf("unknown recipe format %q", format))
		}

		if isJSONNull(raw.Version) {
			return rootRecipeError(ErrInvalidRecipe, errors.New(`invalid recipe field "version": null`))
		}
		if err := json.Unmarshal(raw.Version, &version); err != nil {
			return rootRecipeError(ErrInvalidRecipe, fmt.Errorf(`invalid recipe field "version": %w`, err))
		}
		// A decoder that supports only version N MUST reject a complete
		// version-(N+1) envelope before it decodes any step (replay design
		// §2.1) — this decoder supports 1 and 2, so this is the whole gate:
		// no step in raw.Steps is ever inspected for an unsupported version.
		if version < recipeWireLegacyVersion || version > recipeWireVersion {
			return rootRecipeError(
				ErrUnsupportedRecipeVersion,
				fmt.Errorf("unsupported recipe version %d", version),
			)
		}
		if isJSONNull(raw.Steps) {
			return rootRecipeError(ErrInvalidRecipe, errors.New(`versioned recipe field "steps" must be an array`))
		}
	}

	var steps []Step
	if !isJSONNull(raw.Steps) {
		var rawSteps []json.RawMessage
		if err := json.Unmarshal(raw.Steps, &rawSteps); err != nil {
			return newRecipeDecodeError(-1, "steps", codecJSONError(err))
		}
		steps = make([]Step, 0, len(rawSteps))
		for i, data := range rawSteps {
			var step Step
			if err := json.Unmarshal(data, &step); err != nil {
				return newRecipeDecodeError(i, fmt.Sprintf(`steps[%d]`, i), err)
			}
			// Version 1 is the pre-loft vocabulary (replay design §2.1): a
			// "loft" step under version-1 rules is invalid, never
			// reinterpreted with version-2 rules.
			if version < recipeWireVersion && step.Op == OpLoft {
				return newRecipeDecodeError(
					i, fmt.Sprintf(`steps[%d].op`, i),
					fmt.Errorf(`decad: the %q op requires recipe version %d`, step.Op, recipeWireVersion),
				)
			}
			steps = append(steps, step)
		}
	}
	if steps == nil {
		steps = []Step{}
	}
	*r = Recipe{Steps: steps}
	return nil
}

func rootRecipeError(kind error, err error) error {
	return &RecipeError{
		StepIndex: -1,
		Path:      "$",
		Kind:      kind,
		Err:       err,
	}
}

func isJSONNull(data []byte) bool {
	return bytes.Equal(bytes.TrimSpace(data), []byte("null"))
}

func validateRecipeJSONStructure(data []byte) error {
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.UseNumber()
	if err := scanRecipeJSONValue(dec, true); err != nil {
		return err
	}
	_, err := dec.Token()
	if err == nil {
		return errors.New("recipe contains more than one JSON value")
	}
	if !errors.Is(err, io.EOF) {
		return fmt.Errorf("invalid recipe JSON: %w", err)
	}
	return nil
}

func rejectDuplicateJSONKeys(data []byte) error {
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.UseNumber()
	return scanRecipeJSONValueMode(dec, false, true)
}

// decodeStrictJSON decodes one nested wire value without silently dropping
// unknown or duplicate fields. The step envelope has its own equivalent
// because it needs step-specific error wording, while selectors and their
// predicates share this nested boundary.
func decodeStrictJSON(data []byte, out any, what string) error {
	if err := rejectDuplicateJSONKeys(data); err != nil {
		return err
	}
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.DisallowUnknownFields()
	if err := dec.Decode(out); err != nil {
		return codecJSONErrorAt(data, out, fmt.Errorf(`decad: failed to decode %s: %w`, what, err))
	}
	var trailing json.RawMessage
	if err := dec.Decode(&trailing); err == nil {
		return fmt.Errorf(`decad: %s contains more than one JSON value`, what)
	} else if err != io.EOF {
		return fmt.Errorf(`decad: failed to decode trailing %s JSON: %w`, what, err)
	}
	return nil
}

func scanRecipeJSONValue(dec *json.Decoder, root bool) error {
	return scanRecipeJSONValueMode(dec, root, false)
}

func scanRecipeJSONValueMode(dec *json.Decoder, root, stepRoot bool) error {
	token, err := dec.Token()
	if err != nil {
		return fmt.Errorf("invalid recipe JSON: %w", err)
	}
	delim, ok := token.(json.Delim)
	if !ok {
		return nil
	}

	switch delim {
	case '{':
		seen := make(map[string]struct{})
		var nonCanonicalStepField string
		for dec.More() {
			token, err := dec.Token()
			if err != nil {
				return fmt.Errorf("invalid recipe JSON: %w", err)
			}
			key, ok := token.(string)
			if !ok {
				return errors.New("recipe object key is not a string")
			}
			folded := strings.ToLower(key)
			if _, exists := seen[folded]; exists {
				return fmt.Errorf("duplicate recipe field %q", key)
			}
			seen[folded] = struct{}{}
			if stepRoot {
				if canonical, ok := canonicalStepField(key); ok && key != canonical && nonCanonicalStepField == "" {
					nonCanonicalStepField = key
				}
			}
			if root && key != "format" && key != "version" && key != "steps" {
				return fmt.Errorf("unknown recipe field %q", key)
			}
			if err := scanRecipeJSONValueMode(dec, false, false); err != nil {
				return err
			}
		}
		if _, err := dec.Token(); err != nil {
			return fmt.Errorf("invalid recipe JSON: %w", err)
		}
		if nonCanonicalStepField != "" {
			return fmt.Errorf("non-canonical step field %q", nonCanonicalStepField)
		}
	case '[':
		for dec.More() {
			if err := scanRecipeJSONValueMode(dec, false, false); err != nil {
				return err
			}
		}
		if _, err := dec.Token(); err != nil {
			return fmt.Errorf("invalid recipe JSON: %w", err)
		}
	default:
		return fmt.Errorf("invalid recipe JSON delimiter %q", delim)
	}
	return nil
}

func canonicalStepField(key string) (string, bool) {
	typ := reflect.TypeFor[jsonStep]()
	for i := range typ.NumField() {
		tag := typ.Field(i).Tag.Get("json")
		if comma := strings.IndexByte(tag, ','); comma >= 0 {
			tag = tag[:comma]
		}
		if strings.EqualFold(tag, key) {
			return tag, true
		}
	}
	return "", false
}
