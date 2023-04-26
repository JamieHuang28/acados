
#!/bin/bash
if [ -z "$1" ] || [ -z "$2" ]; then
    echo "usage: sync.sh <source_dir> <target_dir>"
    exit 1
fi

echo "sync acado files from $1 to $2"

# please specify if src and header is divided
is_src_h_divided=0

if [ $is_src_h_divided == 1 ]; then
    target_h_dir=${2}include/
    if [ ! -d "$target_h_dir" ]; then
        # Create the directory
        mkdir "$target_h_dir"
    fi

    target_c_dir=${2}src/
    if [ ! -d "$target_c_dir" ]; then
        # Create the directory
        mkdir "$target_c_dir"
    fi
    # echo "mv ${1}*.c* $target_c_dir"
    # mv ${1}*.c* $target_c_dir
    # echo "mv ${1}*.h* $target_h_dir"
    # mv ${1}*.h* $target_h_dir
else
    target_dir=${2}
    if [ -d "$target_dir" ]; then
        rm -r "$target_dir"
    fi
    mkdir -p "$target_dir"

    echo "mv ${1}* ${target_dir}"
    mv ${1}* ${target_dir} -f
fi

echo 'finished'