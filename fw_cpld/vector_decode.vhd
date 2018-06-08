library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity vector_decode is
	port
	(
		zaddr	 		: in	std_logic_vector(16 downto 0);
		vector		: out	std_logic_vector(7 downto 0);
		int_to_stm	: out	std_logic;
		int_inner	: out	std_logic
	);
end entity;

architecture rtl of vector_decode is

signal mux_lo			: std_logic;
signal mux_hi			: std_logic;
signal int_vector_l	: std_logic_vector(3 downto 0);
signal int_vector_h	: std_logic_vector(3 downto 0);
signal int_vector		: std_logic_vector(7 downto 0);

begin

mux_lo <= (zaddr(15) and zaddr(3));
int_vector_l <= zaddr(7 downto 4) when (mux_lo= '1')	-- L
			  else zaddr(3 downto 0);							-- l

mux_hi <= (zaddr(15) and zaddr(3)) or zaddr(7);
int_vector_h <= zaddr(7 downto 4) when (mux_hi = '0')	-- h
	else zaddr(15 downto 12) when (mux_lo = '0')			-- H
	else zaddr(11 downto 8);									-- L


int_vector <= int_vector_h & int_vector_l;

int_to_stm <= '0' when (int_vector > x"00") else '1';

int_inner <= 	'0' when (((zaddr(16) = '1') and (int_vector = x"1f")) or	-- Kjoy1
								 ((zaddr(16) = '1') and (int_vector = x"7f")) or	-- Fjoy
								 ((zaddr(16) = '1') and (int_vector = x"df")) or	-- Kjoy2
																(int_vector = x"73") or		-- K0
								 								(int_vector = x"b3") or		-- K1
								 								(int_vector = x"d3") or		-- K2
								 								(int_vector = x"e3") or		-- K3
								 								(int_vector = x"7f") or		-- K4
								 								(int_vector = x"bf") or		-- K5
								 								(int_vector = x"df") or		-- K6
								 								(int_vector = x"ef") or		-- K7
								 								(int_vector = x"0f"))		-- version
	else '1';

vector <= int_vector;

end rtl;
