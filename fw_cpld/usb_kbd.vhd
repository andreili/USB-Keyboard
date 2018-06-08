library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity usb_kbd is
	port
	(
		clk		 : in	std_logic;
		
		ziorqn	: in	std_logic;
		zmreqn	: in	std_logic;
		zrdn		: in	std_logic;
		zwrn		: in	std_logic;
		zm1n		: in	std_logic;
		zdata		: inout	std_logic_vector( 7 downto 0);
		zaddr	 	: in	std_logic_vector(15 downto 0);
		zdosn		: in	std_logic;
		tin		: in	std_logic;
		dasel		: in	std_logic;
		f14		: in	std_logic;
		csrn		: in	std_logic;
		iintu		: in	std_logic;
		
		zrstn		: out	std_logic;
		zwaitn	: out	std_logic;
		iorqgen	: out	std_logic;
		iodosn	: out	std_logic;
		int0n		: out	std_logic;
		turbo		: out	std_logic;
		rdrn		: out	std_logic;
		blk		: out	std_logic;
		ointu		: out	std_logic;
		extan		: out	std_logic;
		ula		: out	std_logic;
		da			: inout	std_logic_vector( 7 downto 0);
		
		vector 	: inout	std_logic_vector(7 downto 0);
		sintn		: out	std_logic;	-- сигнал для обработки прерывания
		sintokn	: in	std_logic;	-- обработка прерывания завершена
		dfrstmn	: in	std_logic;	-- данные для регистров в CPLD
		
		zaddr16	: out std_logic;
		int_innero: out std_logic;
		int_st	: out std_logic;
		ctrig	: out std_logic
	);

end entity;

architecture rtl of usb_kbd is

	component vector_decode is
		port
		(
			zaddr	 		: in	std_logic_vector(16 downto 0);
			vector		: out	std_logic_vector(7 downto 0);
			int_to_stm	: out	std_logic;
			int_inner	: out	std_logic
		);
	end component;

	component trig_inner is
		port
		(
			sig_to_tr	: in	std_logic;
			a16			: in	std_logic;
			tin			: in	std_logic;
			vector		: in	std_logic_vector(7 downto 0);
			data			: in	std_logic_vector(7 downto 0);
			odata			: out	std_logic_vector(7 downto 0)
		);
	end component;

signal int_to_stm		: std_logic;
signal int_inner		: std_logic;
signal int_vector		: std_logic_vector(7 downto 0);

signal zaddr16i			: std_logic;
signal zaddr16clk		: std_logic;

signal dfrstmn_in		: std_logic;
signal odata			: std_logic_vector(7 downto 0);

begin

zaddr16clk <= zmreqn or zm1n;

process (zaddr16clk)
begin
	if (rising_edge(zaddr16clk)) then
		zaddr16i <= (not (zdata(2) or zdata(5))) and zdata(0) and zdata(1) and zdata(4) and zdata(6) and zdata(7);
	end if;
end process;

dec: vector_decode
	port map (
		zaddr			=> zaddr16i & zaddr,
		vector		=> int_vector,
		int_to_stm	=> int_to_stm,
		int_inner	=> int_inner
	);

dfrstmn_in <= '0' when ((dfrstmn = '0') and (ziorqn = '1'))
					else '1';
vector <= int_vector when (dfrstmn_in = '1') else (others => 'Z');

trig: trig_inner
	port map (
		sig_to_tr	=> dfrstmn_in,
		a16			=> zaddr16i,
		tin			=> tin,
		vector		=> vector,
		data			=> da,
		odata			=> odata
	);

-- to outputs
process (int_to_stm, sintokn)
begin
	if (sintokn = '0') then
		zwaitn <= 'Z';
	elsif (falling_edge(int_to_stm)) then
		zwaitn <= '0';
	end if;
end process;
--zwaitn <= int_to_stm and sintokn;
sintn <= int_to_stm;

zdata <= odata when ((int_inner = '0') and (ziorqn = '0') and (zrdn = '0'))
	else (others => 'Z');

iorqgen <= int_to_stm and int_inner;

zaddr16 <= zaddr16i;
int_innero <= int_inner;
int_st <= int_to_stm;
ctrig <= zaddr16clk;

end rtl;
