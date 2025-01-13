while test $# -gt 0; do
  case "$1" in
    -h|--help)
      echo "my application!"
      echo " "
      echo "options:"
      echo "-h, --help                show brief help"
      echo "-n, --name=NAME       specify the name of the container"
      echo "-o, --output-dir=DIR      specify a directory to store output in"
      exit 0
      ;;

    -n)
      shift
      if test $# -gt 0; then # If its not empty
        NAME=${1}
        echo ${1}
      else
        echo "nothing specified!"
        exit 1
      fi
      shift
      ;;
    --name*)
      export PROCESS=`echo $1 | sed -e 's/^[^=]*=//g'`
      shift
      ;;

    -o)
      shift
      if test $# -gt 0; then
        export OUTPUT=$1
      else
        echo "nothing specified!"
        exit 1
      fi
      shift
      ;;
    --output-dir*)
      export OUTPUT=`echo $1 | sed -e 's/^[^=]*=//g'`
      shift
      ;;

    *)
      break
      ;;

      
  esac
done