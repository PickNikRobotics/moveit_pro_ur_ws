# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The tokenizer must give the ids Meta's sam3 tokenizer gives, which the merged graph was exported with.

test/data/sam3_reference_tokens.json holds [prompt, ids] pairs from sam3.model.tokenizer_ve.SimpleTokenizer
(context length 32): every exemplars.yaml description, the generic prompts, and edge cases (whitespace,
HTML entities, digits, underscores, contractions, accents, truncation).
"""

import json
import os

import pytest

from sam3_npu_server.tokenizer import Sam3Tokenizer

DATA = os.path.join(os.path.dirname(__file__), "data", "sam3_reference_tokens.json")
with open(DATA) as f:
    REFERENCE = json.load(f)


@pytest.fixture(scope="module")
def tokenizer():
    return Sam3Tokenizer(context_length=32)


@pytest.mark.parametrize("prompt,expected", REFERENCE, ids=[p[:40] for p, _ in REFERENCE])
def test_matches_sam3_reference(tokenizer, prompt, expected):
    assert tokenizer(prompt) == expected


def test_long_prompt_is_truncated_to_end_on_the_end_token(tokenizer):
    ids = tokenizer("a very long prompt " * 20)
    assert len(ids) == 32
    assert ids[0] == tokenizer.start_id
    assert ids[-1] == tokenizer.end_id
