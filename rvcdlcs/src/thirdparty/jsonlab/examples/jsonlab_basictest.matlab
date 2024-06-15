
                            < M A T L A B (R) >
                  Copyright 1984-2016 The MathWorks, Inc.
                   R2016a (9.0.0.341360) 64-bit (glnxa64)
                             February 11, 2016

 
For online documentation, see http://www.mathworks.com/support
For product information, visit www.mathworks.com.
 

	Academic License

>> >> >> >> >> >> >> >> >> >> >> 
%=================================================
>> %  a simple scalar value 
>> %=================================================

>> >> 
data2json =

    3.1416

>> 
ans =

[3.141592654]


>> 
json2data =

    3.1416

>> >> 
%=================================================
>> %  an empty array 
>> %=================================================

>> >> 
data2json =

     []

>> 
ans =

[]


>> 
json2data = 

   Empty cell array: 0-by-1

>> >> 
%=================================================
>> %  an empty string 
>> %=================================================

>> >> 
data2json =

     ''


>> 
ans =

{
	"emptystr":""
}


>> 
json2data = 

    emptystr: [1x0 char]

>> >> 
%=================================================
>> %  a simple row vector 
>> %=================================================

>> >> 
data2json =

     1     2     3

>> 
ans =

[1,2,3]


>> 
json2data =

     1     2     3

>> >> >> 
%=================================================
>> %  a simple column vector 
>> %=================================================

>> >> 
data2json =

     1
     2
     3

>> 
ans =

[
	[1],
	[2],
	[3]
]


>> 
json2data =

     1
     2
     3

>> >> >> 
%=================================================
>> %  a string array 
>> %=================================================

>> >> 
data2json =

AC
EG

>> 
ans =

[
	"AC",
	"EG"
]


>> 
json2data = 

    'AC'    'EG'

>> >> 
%=================================================
>> %  a string with escape symbols 
>> %=================================================

>> >> 
data2json =

AB	CD
one"two

>> 
ans =

{
	"str":"AB\tCD\none\"two"
}


>> 
json2data = 

    str: 'AB	CD...'

>> >> 
%=================================================
>> %  a mix-typed cell 
>> %=================================================

>> >> 
data2json = 

    'a'    [1]    [2x1 double]

>> 
ans =

[
	"a",
	true,
	[
		[2],
		[3]
	]
]


>> 
json2data = 

    'a'    [1]    [2x1 double]

>> >> >> 
%=================================================
>> %  a 3-D array in nested array form
>> %=================================================

>> >> >> 
ans =

[
		[
			[1,9,17,25,33,41],
			[3,11,19,27,35,43],
			[5,13,21,29,37,45],
			[7,15,23,31,39,47]
		],
		[
			[2,10,18,26,34,42],
			[4,12,20,28,36,44],
			[6,14,22,30,38,46],
			[8,16,24,32,40,48]
		]
]


>> 
json2data(:,:,1) =

     1     3     5     7
     2     4     6     8


json2data(:,:,2) =

     9    11    13    15
    10    12    14    16


json2data(:,:,3) =

    17    19    21    23
    18    20    22    24


json2data(:,:,4) =

    25    27    29    31
    26    28    30    32


json2data(:,:,5) =

    33    35    37    39
    34    36    38    40


json2data(:,:,6) =

    41    43    45    47
    42    44    46    48

>> >> >> 
%=================================================
>> %  a 3-D array in annotated array form
>> %=================================================

>> >> >> 
ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[2,4,6],
	"_ArrayData_":[1,9,17,25,33,41,3,11,19,27,35,43,5,13,21,29,37,45,7,15,23,31,39,47,2,10,18,26,34,42,4,12,20,28,36,44,6,14,22,30,38,46,8,16,24,32,40,48]
}


>> 
json2data(:,:,1) =

     1     3     5     7
     2     4     6     8


json2data(:,:,2) =

     9    11    13    15
    10    12    14    16


json2data(:,:,3) =

    17    19    21    23
    18    20    22    24


json2data(:,:,4) =

    25    27    29    31
    26    28    30    32


json2data(:,:,5) =

    33    35    37    39
    34    36    38    40


json2data(:,:,6) =

    41    43    45    47
    42    44    46    48

>> >> >> 
%=================================================
>> %  a 4-D array in annotated array form
>> %=================================================

>> >> >> 
ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[2,4,3,2],
	"_ArrayData_":[1,25,9,33,17,41,3,27,11,35,19,43,5,29,13,37,21,45,7,31,15,39,23,47,2,26,10,34,18,42,4,28,12,36,20,44,6,30,14,38,22,46,8,32,16,40,24,48]
}


>> 
json2data(:,:,1,1) =

     1     3     5     7
     2     4     6     8


json2data(:,:,2,1) =

     9    11    13    15
    10    12    14    16


json2data(:,:,3,1) =

    17    19    21    23
    18    20    22    24


json2data(:,:,1,2) =

    25    27    29    31
    26    28    30    32


json2data(:,:,2,2) =

    33    35    37    39
    34    36    38    40


json2data(:,:,3,2) =

    41    43    45    47
    42    44    46    48

>> >> >> 
%=================================================
>> %  a 3-D array in nested array form (JSONLab 1.9)
>> %=================================================

>> >> >> 
ans =

[
		[
			[1,2],
			[3,4],
			[5,6],
			[7,8]
		],
		[
			[9,10],
			[11,12],
			[13,14],
			[15,16]
		],
		[
			[17,18],
			[19,20],
			[21,22],
			[23,24]
		],
		[
			[25,26],
			[27,28],
			[29,30],
			[31,32]
		],
		[
			[33,34],
			[35,36],
			[37,38],
			[39,40]
		],
		[
			[41,42],
			[43,44],
			[45,46],
			[47,48]
		]
]


>> 
json2data(:,:,1) =

     1     3     5     7
     2     4     6     8


json2data(:,:,2) =

     9    11    13    15
    10    12    14    16


json2data(:,:,3) =

    17    19    21    23
    18    20    22    24


json2data(:,:,4) =

    25    27    29    31
    26    28    30    32


json2data(:,:,5) =

    33    35    37    39
    34    36    38    40


json2data(:,:,6) =

    41    43    45    47
    42    44    46    48

>> >> >> 
%=================================================
>> %  a 3-D array in annotated array form (JSONLab 1.9 or earlier)
>> %=================================================

>> >> >> 
ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[2,4,6],
	"_ArrayData_":[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48]
}


>> 
json2data(:,:,1) =

     1     3     5     7
     2     4     6     8


json2data(:,:,2) =

     9    11    13    15
    10    12    14    16


json2data(:,:,3) =

    17    19    21    23
    18    20    22    24


json2data(:,:,4) =

    25    27    29    31
    26    28    30    32


json2data(:,:,5) =

    33    35    37    39
    34    36    38    40


json2data(:,:,6) =

    41    43    45    47
    42    44    46    48

>> >> >> 
%=================================================
>> %  a complex number
>> %=================================================

>> >> 
data2json =

   1.0000 + 2.0000i

>> 
ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[1,1],
	"_ArrayIsComplex_":true,
	"_ArrayData_":[
		[1],
		[2]
	]
}


>> 
json2data =

   1.0000 + 2.0000i

>> >> >> 
%=================================================
>> %  a complex matrix
>> %=================================================

>> >> >> 
data2json =

  35.0000 +26.0000i   1.0000 +19.0000i   6.0000 +24.0000i
   3.0000 +21.0000i  32.0000 +23.0000i   7.0000 +25.0000i
  31.0000 +22.0000i   9.0000 +27.0000i   2.0000 +20.0000i
   8.0000 +17.0000i  28.0000 +10.0000i  33.0000 +15.0000i
  30.0000 +12.0000i   5.0000 +14.0000i  34.0000 +16.0000i
   4.0000 +13.0000i  36.0000 +18.0000i  29.0000 +11.0000i

>> 
ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[6,3],
	"_ArrayIsComplex_":true,
	"_ArrayData_":[
		[35,1,6,3,32,7,31,9,2,8,28,33,30,5,34,4,36,29],
		[26,19,24,21,23,25,22,27,20,17,10,15,12,14,16,13,18,11]
	]
}


>> 
json2data =

  35.0000 +26.0000i   1.0000 +19.0000i   6.0000 +24.0000i
   3.0000 +21.0000i  32.0000 +23.0000i   7.0000 +25.0000i
  31.0000 +22.0000i   9.0000 +27.0000i   2.0000 +20.0000i
   8.0000 +17.0000i  28.0000 +10.0000i  33.0000 +15.0000i
  30.0000 +12.0000i   5.0000 +14.0000i  34.0000 +16.0000i
   4.0000 +13.0000i  36.0000 +18.0000i  29.0000 +11.0000i

>> >> >> 
%=================================================
>> %  MATLAB special constants
>> %=================================================

>> >> 
data2json =

   NaN   Inf  -Inf

>> 
ans =

{
	"specials":["_NaN_","_Inf_","-_Inf_"]
}


>> 
json2data = 

    specials: [NaN Inf -Inf]

>> >> >> 
%=================================================
>> %  a real sparse matrix
>> %=================================================

>> >> 
data2json =

   (1,2)       0.6557
   (9,2)       0.7577
   (3,5)       0.8491
  (10,5)       0.7431
  (10,8)       0.3922
   (7,9)       0.6787
   (2,10)      0.0357
   (6,10)      0.9340
  (10,10)      0.6555

>> 
ans =

{
	"sparse":{
		"_ArrayType_":"double",
		"_ArraySize_":[10,10],
		"_ArrayIsSparse_":true,
		"_ArrayData_":[
			[1,9,3,10,10,7,2,6,10],
			[2,2,5,5,8,9,10,10,10],
			[0.655740699156586837,0.757740130578333448,0.849129305868777107,0.743132468124916179,0.392227019534168164,0.678735154857773471,0.0357116785741895537,0.933993247757550549,0.655477890177556644]
		]
	}
}


>> 
json2data = 

    sparse: [10x10 double]

>> >> >> 
%=================================================
>> %  a complex sparse matrix
>> %=================================================

>> >> >> 
data2json =

   (2,1)      0.6551 - 0.6551i
   (1,2)      0.7547 - 0.7547i
   (1,4)      0.2760 - 0.2760i
   (7,5)      0.4984 - 0.4984i
   (8,5)      0.9597 - 0.9597i
   (9,5)      0.3404 - 0.3404i
   (4,7)      0.1190 - 0.1190i
   (1,8)      0.6797 - 0.6797i
   (3,8)      0.1626 - 0.1626i
  (10,8)      0.5853 - 0.5853i

>> 
ans =

{
	"complex_sparse":{
		"_ArrayType_":"double",
		"_ArraySize_":[10,10],
		"_ArrayIsComplex_":true,
		"_ArrayIsSparse_":true,
		"_ArrayData_":[
			[2,1,1,7,8,9,4,1,3,10],
			[1,2,4,5,5,5,7,8,8,8],
			[0.655098003973840659,0.754686681982360885,0.276025076998578367,0.498364051982142953,0.959743958516081075,0.340385726666133204,0.118997681558376645,0.679702676853674803,0.162611735194630569,0.585267750979777346],
			[-0.655098003973840659,-0.754686681982360885,-0.276025076998578367,-0.498364051982142953,-0.959743958516081075,-0.340385726666133204,-0.118997681558376645,-0.679702676853674803,-0.162611735194630569,-0.585267750979777346]
		]
	}
}


>> 
json2data = 

    complex_sparse: [10x10 double]

>> >> >> 
%=================================================
>> %  an all-zero sparse matrix
>> %=================================================

>> >> >> 
ans =

{
	"all_zero_sparse":{
		"_ArrayType_":"double",
		"_ArraySize_":[2,3],
		"_ArrayIsSparse_":true,
		"_ArrayData_":[]
	}
}


>> 
json2data = 

    all_zero_sparse: [2x3 double]

>> >> >> 
%=================================================
>> %  an empty sparse matrix
>> %=================================================

>> >> >> 
ans =

{
	"empty_sparse":{
		"_ArrayType_":"double",
		"_ArraySize_":[0,0],
		"_ArrayIsSparse_":true,
		"_ArrayData_":[]
	}
}


>> 
json2data = 

    empty_sparse: []

>> >> >> 
%=================================================
>> %  an empty 0-by-0 real matrix
>> %=================================================

>> >> >> 
ans =

{
	"empty_0by0_real":[]
}


>> 
json2data = 

    empty_0by0_real: {0x1 cell}

>> >> 
%=================================================
>> %  an empty 0-by-3 real matrix
>> %=================================================

>> >> >> 
ans =

{
	"empty_0by3_real":{
		"_ArrayType_":"double",
		"_ArraySize_":[0,3],
		"_ArrayData_":[]
	}
}


>> 
json2data = 

    empty_0by3_real: [0x3 double]

>> >> >> 
%=================================================
>> %  a sparse real column vector
>> %=================================================

>> >> >> 
ans =

{
	"sparse_column_vector":{
		"_ArrayType_":"double",
		"_ArraySize_":[5,1],
		"_ArrayIsSparse_":true,
		"_ArrayData_":[
			[2,4,5],
			[3,1,4]
		]
	}
}


>> 
json2data = 

    sparse_column_vector: [5x1 double]

>> >> 
%=================================================
>> %  a sparse complex column vector
>> %=================================================

>> >> >> 
ans =

{
	"complex_sparse_column_vector":{
		"_ArrayType_":"double",
		"_ArraySize_":[5,1],
		"_ArrayIsComplex_":true,
		"_ArrayIsSparse_":true,
		"_ArrayData_":[
			[2,4,5],
			[3,1,4],
			[-3,-1,-4]
		]
	}
}


>> 
json2data = 

    complex_sparse_column_vector: [5x1 double]

>> >> 
%=================================================
>> %  a sparse real row vector
>> %=================================================

>> >> >> 
ans =

{
	"sparse_row_vector":{
		"_ArrayType_":"double",
		"_ArraySize_":[1,5],
		"_ArrayIsSparse_":true,
		"_ArrayData_":[
			[2,4,5],
			[3,1,4]
		]
	}
}


>> 
json2data = 

    sparse_row_vector: [0 3 0 1 4]

>> >> 
%=================================================
>> %  a sparse complex row vector
>> %=================================================

>> >> >> 
ans =

{
	"complex_sparse_row_vector":{
		"_ArrayType_":"double",
		"_ArraySize_":[1,5],
		"_ArrayIsComplex_":true,
		"_ArrayIsSparse_":true,
		"_ArrayData_":[
			[2,4,5],
			[3,1,4],
			[-3,-1,-4]
		]
	}
}


>> 
json2data = 

    complex_sparse_row_vector: [1x5 double]

>> >> 
%=================================================
>> %  a structure
>> %=================================================

>> >> 
data2json = 

        name: 'Think Different'
        year: 1997
       magic: [3x3 double]
     misfits: [Inf NaN]
    embedded: [1x1 struct]

>> 
ans =

{
	"astruct":{
		"name":"Think Different",
		"year":1997,
		"magic":[
			[8,1,6],
			[3,5,7],
			[4,9,2]
		],
		"misfits":["_Inf_","_NaN_"],
		"embedded":{
			"left":true,
			"right":false
		}
	}
}


>> 
json2data = 

    astruct: [1x1 struct]

>> 
ans =

logical

>> >> 
%=================================================
>> %  a structure array
>> %=================================================

>> >> >> >> >> 
ans =

{
	"Supreme Commander":[
		{
			"name":"Nexus Prime",
			"rank":9
		},
		{
			"name":"Sentinel Prime",
			"rank":9
		},
		{
			"name":"Optimus Prime",
			"rank":9
		}
	]
}


>> 
json2data = 

    Supreme_0x20_Commander: [1x3 struct]

>> >> 
%=================================================
>> %  a cell array
>> %=================================================

>> >> >> >> >> 
data2json = 

    [1x1 struct]
    [1x1 struct]
    [1x4 double]

>> 
ans =

{
	"debian":[
			[
				{
					"buzz":1.10,
					"rex":1.20,
					"bo":1.30,
					"hamm":2.00,
					"slink":2.10,
					"potato":2.20,
					"woody":3.00,
					"sarge":3.10,
					"etch":4.00,
					"lenny":5.00,
					"squeeze":6.00,
					"wheezy":7.00
				}
			],
			[
				{
					"Ubuntu":[
						"Kubuntu",
						"Xubuntu",
						"Lubuntu"
					]
				}
			],
			[
				[10.04,10.10,11.04,11.10]
			]
	]
}


>> 
json2data = 

    debian: {[1x1 struct]  [1x1 struct]  [10.0400 10.1000 11.0400 11.1000]}

>> >> 
%=================================================
>> %  invalid field-name handling
>> %=================================================

>> >> 
json2data = 

               ValidName: 1
       x0x5F_InvalidName: 2
       x0x3A_Field_0x3A_: 3
    x0xE9A1B9__0xE79BAE_: '绝密'

>> >> 
%=================================================
>> %  a function handle
>> %=================================================

>> >> 
data2json = 

    @(x)x+1

>> 
ans =

{
	"handle":{
		"function":"@(x)x+1",
		"type":"anonymous",
		"file":"",
		"workspace":[
			{
			}
		],
		"within_file_path":"__base_function"
	}
}


>> 
json2data = 

    handle: [1x1 struct]

>> >> 
%=================================================
>> %  a 2D cell array
>> %=================================================

>> >> >> 
ans =

{
	"data2json":[
			[
				[
					1,
					[
						2,
						3
					]
				],
				[
					4,
					5
				],
				[
					6
				]
			],
			[
				[
					7
				],
				[
					8,
					9
				],
				[
					10
				]
			]
	]
}


>> 
json2data = 

    data2json: {{1x3 cell}  {1x3 cell}}

>> >> 
%=================================================
>> %  a 2D struct array
>> %=================================================

>> >> 
data2json = 

2x3 struct array with fields:

    idx
    data

>> >> 
ans =

{
	"data2json":[
		[
			{
				"idx":1,
				"data":"structs"
			},
			{
				"idx":2,
				"data":"structs"
			}
		],
		[
			{
				"idx":3,
				"data":"structs"
			},
			{
				"idx":4,
				"data":"structs"
			}
		],
		[
			{
				"idx":5,
				"data":"structs"
			},
			{
				"idx":6,
				"data":"structs"
			}
		]
	]
}


>> 
json2data = 

    data2json: [2x3 struct]

>> >> >> 
%=================================================
%  datetime object 
%=================================================


data2json = 

   08-Apr-2015   09-May-2015


ans =

[
	{
		"Format":"dd-MMM-uuuu",
		"TimeZone":"",
		"Year":2015,
		"Month":4,
		"Day":8,
		"Hour":0,
		"Minute":0,
		"Second":0,
		"SystemTimeZone":"America\/New_York"
	},
	{
		"Format":"dd-MMM-uuuu",
		"TimeZone":"",
		"Year":2015,
		"Month":5,
		"Day":9,
		"Hour":0,
		"Minute":0,
		"Second":0,
		"SystemTimeZone":"America\/New_York"
	}
]



json2data = 

1x2 struct array with fields:

    Format
    TimeZone
    Year
    Month
    Day
    Hour
    Minute
    Second
    SystemTimeZone

>> >> 
%=================================================
%  a container.Maps object 
%=================================================


data2json = 

  Map with properties:

        Count: 3
      KeyType: char
    ValueType: double


ans =

{
	"Andy":21,
	"Om":22,
	"William":21
}



json2data = 

       Andy: 21
         Om: 22
    William: 21

>> >> 
%=================================================
%  a table object 
%=================================================


data2json = 

      Names      Age
    _________    ___

    'Andy'       21 
    'William'    21 
    'Om'         22 


ans =

{
	"table":{
		"_TableCols_":[
			"Names",
			"Age"
		],
		"_TableRows_":[],
		"_TableRecords_":[
				[
					"Andy",
					21
				],
				[
					"William",
					21
				],
				[
					"Om",
					22
				]
		]
	}
}



json2data = 

    table: [3x2 table]

>> >> 
%=================================================
%  use _ArrayShape_ 
%=================================================


data2json =

     1     6    11     0     0     0     0     0
     2     7    12    17     0     0     0     0
     3     8    13    18    23     0     0     0
     4     9    14    19    24    29     0     0
     0    10    15    20    25    30    35     0


ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[5,8],
	"_ArrayZipSize_":[6,5],
	"_ArrayShape_":[
		"band",
		2,
		3
	],
	"_ArrayData_":[11,17,23,29,35,6,12,18,24,30,1,7,13,19,25,0,2,8,14,20,0,0,3,9,15,0,0,0,4,10]
}



json2data =

     1     6    11     0     0     0     0     0
     2     7    12    17     0     0     0     0
     3     8    13    18    23     0     0     0
     4     9    14    19    24    29     0     0
     0    10    15    20    25    30    35     0


ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[5,8],
	"_ArrayZipSize_":[4,5],
	"_ArrayShape_":[
		"lowerband",
		3
	],
	"_ArrayData_":[1,7,13,19,25,0,2,8,14,20,0,0,3,9,15,0,0,0,4,10]
}



json2data =

     1     0     0     0     0     0     0     0
     2     7     0     0     0     0     0     0
     3     8    13     0     0     0     0     0
     4     9    14    19     0     0     0     0
     0    10    15    20    25     0     0     0


ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[5,8],
	"_ArrayIsComplex_":true,
	"_ArrayZipSize_":[2,3,5],
	"_ArrayShape_":[
		"upperband",
		2
	],
	"_ArrayData_":[11,17,23,29,35,6,12,18,24,30,1,7,13,19,25,11,17,23,29,35,6,12,18,24,30,1,7,13,19,25]
}



json2data =

  Columns 1 through 4

   1.0000 + 1.0000i   6.0000 + 6.0000i  11.0000 +11.0000i   0.0000 + 0.0000i
   0.0000 + 0.0000i   7.0000 + 7.0000i  12.0000 +12.0000i  17.0000 +17.0000i
   0.0000 + 0.0000i   0.0000 + 0.0000i  13.0000 +13.0000i  18.0000 +18.0000i
   0.0000 + 0.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i  19.0000 +19.0000i
   0.0000 + 0.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i

  Columns 5 through 8

   0.0000 + 0.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i
   0.0000 + 0.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i
  23.0000 +23.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i
  24.0000 +24.0000i  29.0000 +29.0000i   0.0000 + 0.0000i   0.0000 + 0.0000i
  25.0000 +25.0000i  30.0000 +30.0000i  35.0000 +35.0000i   0.0000 + 0.0000i


ans =

{
	"_ArrayType_":"int8",
	"_ArraySize_":[5,8],
	"_ArrayShape_":"diag",
	"_ArrayData_":[1,7,13,19,25]
}



json2data =

    1    0    0    0    0    0    0    0
    0    7    0    0    0    0    0    0
    0    0   13    0    0    0    0    0
    0    0    0   19    0    0    0    0
    0    0    0    0   25    0    0    0


ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[5,5],
	"_ArrayZipSize_":[4,5],
	"_ArrayShape_":[
		"lowersymmband",
		3
	],
	"_ArrayData_":[2,14,26,38,50,0,8,20,32,44,0,0,14,26,38,0,0,0,4,10]
}



json2data =

     2     8    14     4     0
     8    14    20    26    10
    14    20    26    32    38
     4    26    32    38    44
     0    10    38    44    50

>> >> 
%=================================================
%  a 2-D array in compressed array format
%=================================================


ans =

{
	"_ArrayType_":"double",
	"_ArraySize_":[20,10],
	"_ArrayZipSize_":[1,200],
	"_ArrayZipType_":"zlib",
	"_ArrayZipData_":"eJxjYACBD/YMNAGj5o6aO2ruKBgFgwtQL10DAMHODQY=
"
}



json2data =

     1     0     0     0     0     0     0     0     0     0
     0     1     0     0     0     0     0     0     0     0
     0     0     1     0     0     0     0     0     0     0
     0     0     0     1     0     0     0     0     0     0
     0     0     0     0     1     0     0     0     0     0
     0     0     0     0     0     1     0     0     0     0
     0     0     0     0     0     0     1     0     0     0
     0     0     0     0     0     0     0     1     0     0
     0     0     0     0     0     0     0     0     1     0
     0     0     0     0     0     0     0     0     0     1
     0     0     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0     0     0
     0     0     0     0     0     0     0     0     0     0
     1     0     0     0     0     0     0     0     0     0

>> >> >> >> 