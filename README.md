## Introduction

This package consists of the hardware implementation of BWA-MEM smith-waterman and the software BWA which links Intel AALSDK. Please make sure you have properly unpacked and compiled the AALSDK. The code has been tested against AALSDK 4.1.0.

## Getting started

Checkout the github repo `bwa-mem-sw` for RTLs and `bwa-mem-quickassist` for the software host.

	git clone https://github.com/peterpengwei/bwa-mem-sw.git
	git clone https://github.com/peterpengwei/bwa-mem-quickassist.git --brach smith-waterman

### Build the software host
	
Please change the `AAL_SRCDIR` and `AAL_BUILDDIR` in **bwa-mem-quickassist/bwa-0.7.8/Makefile** and point them to your own copy of AALSDK.

	cd bwa-mem-quickassist/bwa-0.7.8/
	make
	
This should build the software host for you. Next we need to change the script to run the program. 

	cd ..
	vi pipeline.sh

Please change `BWA` and `ODIR` to point to your executable and a writable directory respectively. Change `ROOT` if the input and reference files are moved in our server.

Notice in the command in **pipeline.sh**

	$BWA --target=ASE mem -t $NTHREAD -b $NBATCH_SIZE -Ma -R $HDR $FASTA $IN1
	
only the second argument `--target` is passed to the AAL host and the rest is for BWA. There are two options for it:

*	`ASE` for simulation using ASE
*	`Direct` for direct interaction with the FPGA

No matter using which mode, the hardware end need to be running first as the software will try to detect the presence of hardware upon launching.

### Compile the hardware implementation (for simulation)

In this step, we need to tell the ASE simulator the location of our RTL so that it can compile them into the simulation.

First, go to the directory of your ASE.

	cd /path/to/ase

Then use the python script to configure the environment, and compile.

	python scripts/gen_afu_env.py /path/to/rtl/bwa-mem-sw
	make

Now the simulator is ready. Launch the hardware simulation by

	make sim
	
The terminal that will be running the software need to set the ASE environment.

	export ASE_WORKDIR=/path/to/ase

The software need the flag `--target=ASE` in order to interact with the simulator. The ASE transaction history will be stored in **transaction.tsv** and the waveform is stored in **inter.vpd** in the ase directory.

### Synthesize the hardware implementation (for on-board test)

This step needs to be performed on JC1 or other servers with Quartus installed.


For synthesis, we use an afu template and redirect source files to our RTLs. You can either copy it from my directory (**/curr/haoyc/afu-quickassist/afu_template**) or ask from Prof. Lei.

	cd afu_template
	vi set_env
	
Again, please change the AAL environment `AAL_SRC` `AAL_SDK` `AAL_RTL` `AAL_PAR` and `LD_LIBRARY_PATH` accordingly.

	csh
	source set_env
	setenv WORKDIR $PWD/fpga/IP

Modify the script to create a new project for the synthesis.

	cd fpga/build
	vi run_ca.sh

Make sure the `DEFAULT_PROJ_NAME` does conflict with any folder names within the same directory. 
	
Next we need to link our RTLs. Please find the **afu_par_files.tcl** under the RTL folder, and change the directory to yours. Now back to the **afu_template/fpga/build** directory, we can start the synthesis.

	./run_ca.sh -ccis_afu /path/to/rtl/afu_par_files.tcl
	
The wait shall begin.

### Test using the Quickassist machine

The test involves loading the bitstream onto the FPGA and running the software host on the same machine. 

Log on to JC1 with `ssh -X` as we need to start Quartus GUI. 

	cd /path/to/afu_template
	csh
	source set_env
	
Go to the project directory and launch Quartus.

	cd fpga/build/your_project/your_project_seed0/
	quartus *.qpf
	
Then we can use the programmer in Quartus to program the FPGA.

Now log on to gene3 and install the CCI driver.

	cd /INTEL_QAFPGA_Starter_Kit_4_1_0/aalsdk_splrm‐4.1.0/aalsdk_splrm‐4.1.0/
	export DESTDIR=$PWD/myinstall
	sudo ./aalkernel/insdrv direct‐jkt

Install the AAL driver.

	export AAL_REGISTRAR_DATABASE_PATH=$PWD/myinstall/usr/local/share/aalsdk/RegistrarRepository/linux/
	export LD_LIBRARY_PATH=$PWD/myinstall/usr/local/lib

Then we can launch the software host with `--target=Direct` for testing.



## Issues

* Current 15 PEs per PE-array 4 PE-arrays (60 PEs in total) version still has timing violation. Could be solved with reduced PEs per PE-array (maybe 12 or 10).
* Reprogramming of FPGA will cause core panic, so we either we need to reboot the kernel after reprogramming or reprogram the FPGA immediately after a reboot (before OS acquires the device mapping). Please see Di, Young or Bug if you need help.