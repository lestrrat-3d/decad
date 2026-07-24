package decad

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
)

const recipeWireFormat = "decad.recipe"
const recipeWireVersion = 1

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

type recipeWireV1 struct {
	Format  string `json:"format"`
	Version int    `json:"version"`
	Steps   []Step `json:"steps"`
}

type rawRecipeWire struct {
	Format  json.RawMessage `json:"format"`
	Version json.RawMessage `json:"version"`
	Steps   json.RawMessage `json:"steps"`
}

// MarshalJSON writes the canonical version 1 recipe envelope. Nil and empty
// step slices both encode as an array.
func (r Recipe) MarshalJSON() ([]byte, error) {
	steps := r.Steps
	if steps == nil {
		steps = []Step{}
	}
	data, err := json.Marshal(recipeWireV1{
		Format:  recipeWireFormat,
		Version: recipeWireVersion,
		Steps:   steps,
	})
	if err != nil {
		return nil, rootRecipeError(ErrInvalidRecipe, fmt.Errorf("failed to encode recipe: %w", err))
	}
	return data, nil
}

// UnmarshalJSON accepts the canonical version 1 envelope and the legacy
// unversioned {"steps": ...} form. It rejects unknown root fields and
// duplicate keys before typed step decoding.
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
		var version int
		if err := json.Unmarshal(raw.Version, &version); err != nil {
			return rootRecipeError(ErrInvalidRecipe, fmt.Errorf(`invalid recipe field "version": %w`, err))
		}
		if version != recipeWireVersion {
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
		if err := json.Unmarshal(raw.Steps, &steps); err != nil {
			return rootRecipeError(ErrInvalidRecipe, fmt.Errorf(`failed to decode recipe field "steps": %w`, err))
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

func scanRecipeJSONValue(dec *json.Decoder, root bool) error {
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
		for dec.More() {
			token, err := dec.Token()
			if err != nil {
				return fmt.Errorf("invalid recipe JSON: %w", err)
			}
			key, ok := token.(string)
			if !ok {
				return errors.New("recipe object key is not a string")
			}
			if _, exists := seen[key]; exists {
				return fmt.Errorf("duplicate recipe field %q", key)
			}
			seen[key] = struct{}{}
			if root && key != "format" && key != "version" && key != "steps" {
				return fmt.Errorf("unknown recipe field %q", key)
			}
			if err := scanRecipeJSONValue(dec, false); err != nil {
				return err
			}
		}
		if _, err := dec.Token(); err != nil {
			return fmt.Errorf("invalid recipe JSON: %w", err)
		}
	case '[':
		for dec.More() {
			if err := scanRecipeJSONValue(dec, false); err != nil {
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
