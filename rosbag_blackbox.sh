#!/usr/bin/env bash

set -euo pipefail

BLACKBOX_DIR=${BLACKBOX_DIR:-${HOME}"/rosbag_blackbox"}
TOPICS_DIR=${TOPICS_DIR:-${BLACKBOX_DIR}"/topics.d"}

BAG_NAME="rosbag2"
TOPIC_FILES=()

# Function to list available topic files
list_topic_files() {
  if [[ ! -d "$TOPICS_DIR" ]]; then
    echo "Error: Directory '$TOPICS_DIR' does not exist." >&2
    exit 1
  fi

  shopt -s nullglob
  local available_files=("$TOPICS_DIR"/*)
  
  if [[ ${#available_files[@]} -eq 0 ]]; then
    echo "No topic files found in '$TOPICS_DIR'."
    exit 0
  fi

  echo "Available topic files in '$TOPICS_DIR':"
  for f in "${available_files[@]}"; do
    [[ -f "$f" ]] && echo "  - $(basename "$f")"
  done
  exit 0
}

# Parse flags
while getopts ":t:o:h" opt; do
  case "$opt" in
    t)
      if [[ "$OPTARG" == "?" ]]; then
        list_topic_files
      fi
      TOPIC_FILES+=("$OPTARG")
      ;;
    o)
      BAG_NAME="$OPTARG"
      ;;
    h)
      echo "Usage: $0 [-t <topic_file>|?]... [-o <bag_name>]" >&2
      exit 0
      ;;
    \?)
      echo "Error: Invalid option -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Error: Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND - 1))

AUTO_NAME="${BAG_NAME}_$(date +'%Y_%m_%d-%H_%M_%S')"
DESTINATION_PATH="${BLACKBOX_DIR}/${AUTO_NAME}"

if [[ ! -d "$TOPICS_DIR" ]]; then
  echo "Error: Directory '$TOPICS_DIR' does not exist." >&2
  exit 1
fi

shopt -s nullglob

raw_topics=()
files=()

# Resolve targeted files or default to all in TOPICS_DIR
if [[ ${#TOPIC_FILES[@]} -gt 0 ]]; then
  for item in "${TOPIC_FILES[@]}"; do
    target_file="${TOPICS_DIR}/${item}"
    if [[ -f "$target_file" ]]; then
      files+=("$target_file")
    elif [[ -f "$item" ]]; then
      files+=("$item")
    else
      echo "Error: Topic file '$item' not found in '$TOPICS_DIR' or current directory." >&2
      exit 1
    fi
  done
else
  files=("$TOPICS_DIR"/*)
fi

for file in "${files[@]}"; do
  if [[ -f "$file" ]]; then
    while IFS= read -r line || [[ -n "$line" ]]; do
      # Strip leading/trailing whitespace
      line="$(echo "$line" | xargs)"
      # Skip empty lines or comment lines
      [[ -z "$line" || "$line" =~ ^# ]] && continue
      raw_topics+=("$line")
    done < "$file"
  fi
done

# Deduplicate and sort topics
topics=()
if [[ ${#raw_topics[@]} -gt 0 ]]; then
  mapfile -t topics < <(printf "%s\n" "${raw_topics[@]}" | sort -u)
fi

if [[ ${#topics[@]} -eq 0 ]]; then
  echo "Error: No valid topics found." >&2
  exit 1
fi

echo "Starting recording for ${#topics[@]} unique topic(s)..."

exec ros2 bag record \
  -s mcap \
  -o "${DESTINATION_PATH}" \
  --compression-mode file \
  --compression-format zstd \
  --topics "${topics[@]}"