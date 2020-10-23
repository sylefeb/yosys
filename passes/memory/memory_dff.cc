/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Copyright (C) 2012  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <algorithm>
#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "kernel/ffinit.h"
#include "kernel/mem.h"
#include "kernel/ff.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct MemoryDffWorker
{
	Module *module;
	SigMap sigmap;

	dict<SigBit, std::pair<Cell*, int>> dff_driver;
	dict<SigBit, pool<std::pair<Cell*, int>>> dff_sink;
	dict<SigBit, int> sigbit_users_count;
	pool<std::pair<Cell*, int>> fwd_merged_bits;
	pool<std::pair<Cell*, int>> rev_merged_bits;
	FfInitVals initvals;

	MemoryDffWorker(Module *module) : module(module), sigmap(module)
	{
		initvals.set(&sigmap, module);
	}

	bool find_sig_before_dff(RTLIL::SigSpec &sig, RTLIL::SigSpec &clk, bool &clk_polarity)
	{
		sigmap.apply(sig);

		dict<SigBit, SigBit> cache;

		for (auto &bit : sig)
		{
			if (cache.count(bit)) {
				bit = cache[bit];
				continue;
			}

			if (bit.wire == NULL)
				continue;

			if (initvals(bit) != State::Sx)
				return false;

			if (!dff_driver.count(bit))
				return false;

			Cell *cell;
			int idx;
			std::tie(cell, idx) = dff_driver[bit];
			if (fwd_merged_bits.count(std::make_pair(cell, idx)))
				return false;

			if (cell->type.in(ID($adff), ID($adffe)))
				return false;

			SigSpec this_clk = cell->getPort(ID::CLK);
			bool this_clk_polarity = cell->parameters[ID::CLK_POLARITY].as_bool();

			if (clk != RTLIL::SigSpec(RTLIL::State::Sx)) {
				if (this_clk != clk)
					return false;
				if (this_clk_polarity != clk_polarity)
					return false;
			}

			RTLIL::SigSpec d = cell->getPort(ID::D)[idx];

			if (cell->type == ID($sdffce)) {
				SigSpec rbit = cell->parameters[ID::SRST_VALUE][idx];
				if (cell->parameters[ID::SRST_POLARITY].as_bool())
					d = module->Mux(NEW_ID, d, rbit, cell->getPort(ID::SRST));
				else
					d = module->Mux(NEW_ID, rbit, d, cell->getPort(ID::SRST));
			}

			if (cell->type.in(ID($dffe), ID($sdffe), ID($sdffce))) {
				if (cell->parameters[ID::EN_POLARITY].as_bool())
					d = module->Mux(NEW_ID, bit, d, cell->getPort(ID::EN));
				else
					d = module->Mux(NEW_ID, d, bit, cell->getPort(ID::EN));
			}

			if (cell->type.in(ID($sdff), ID($sdffe))) {
				SigSpec rbit = cell->parameters[ID::SRST_VALUE][idx];
				if (cell->parameters[ID::SRST_POLARITY].as_bool())
					d = module->Mux(NEW_ID, d, rbit, cell->getPort(ID::SRST));
				else
					d = module->Mux(NEW_ID, rbit, d, cell->getPort(ID::SRST));
			}

			cache[bit] = d;
			bit = d;
			clk = this_clk;
			clk_polarity = this_clk_polarity;
			rev_merged_bits.insert(std::make_pair(cell, idx));
		}

		return true;
	}

	bool find_sig_after_ff(RTLIL::SigSpec sig, FfData &ff, pool<std::pair<Cell *, int>> &disconnect)
	{
		sigmap.apply(sig);

		for (auto &bit : sig)
		{
			if (bit.wire == NULL || sigbit_users_count[bit] == 0) {
				ff.width++;
				ff.sig_q.append(bit);
				ff.sig_d.append(State::Sx);
				ff.val_init.bits.push_back(State::Sx);
				ff.val_srst.bits.push_back(State::Sx);
				ff.val_arst.bits.push_back(State::Sx);
				continue;
			}

			if (sigbit_users_count[bit] != 1)
				return false;

			auto &sinks = dff_sink[bit];
			if (sinks.size() != 1)
				return false;

			Cell *cell;
			int idx;
			std::tie(cell, idx) = *sinks.begin();
			disconnect.insert(std::make_pair(cell, idx));

			if (rev_merged_bits.count(std::make_pair(cell, idx)))
				return false;

			FfData cur_ff(&initvals, cell);

			if (!ff.has_clk) {
				ff.sig_clk = cur_ff.sig_clk;
				ff.sig_en = cur_ff.sig_en;
				ff.sig_srst = cur_ff.sig_srst;
				ff.sig_arst = cur_ff.sig_arst;
				ff.has_clk = cur_ff.has_clk;
				ff.has_en = cur_ff.has_en;
				ff.has_srst = cur_ff.has_srst;
				ff.has_arst = cur_ff.has_arst;
				ff.ce_over_srst = cur_ff.ce_over_srst;
				ff.pol_clk = cur_ff.pol_clk;
				ff.pol_en = cur_ff.pol_en;
				ff.pol_arst = cur_ff.pol_arst;
				ff.pol_srst = cur_ff.pol_srst;
			} else {
				if (ff.sig_clk != cur_ff.sig_clk)
					return false;
				if (ff.pol_clk != cur_ff.pol_clk)
					return false;
				if (ff.has_en != cur_ff.has_en)
					return false;
				if (ff.has_srst != cur_ff.has_srst)
					return false;
				if (ff.has_arst != cur_ff.has_arst)
					return false;
				if (ff.has_en) {
					if (ff.sig_en != cur_ff.sig_en)
						return false;
					if (ff.pol_en != cur_ff.pol_en)
						return false;
				}
				if (ff.has_srst) {
					if (ff.sig_srst != cur_ff.sig_srst)
						return false;
					if (ff.pol_srst != cur_ff.pol_srst)
						return false;
					if (ff.has_en && ff.ce_over_srst != cur_ff.ce_over_srst)
						return false;
				}
				if (ff.has_arst) {
					if (ff.sig_arst != cur_ff.sig_arst)
						return false;
					if (ff.pol_arst != cur_ff.pol_arst)
						return false;
				}
			}

			ff.width++;
			ff.sig_d.append(cur_ff.sig_d[idx]);
			ff.sig_q.append(cur_ff.sig_q[idx]);
			ff.val_arst.bits.push_back(ff.has_arst ? cur_ff.val_arst[idx] : State::Sx);
			ff.val_srst.bits.push_back(ff.has_srst ? cur_ff.val_srst[idx] : State::Sx);
			ff.val_init.bits.push_back(cur_ff.val_init[idx]);
		}

		return true;
	}

	void handle_wr_port(Mem &mem, int idx)
	{
		auto &port = mem.wr_ports[idx];

		log("Checking write port `%s'[%d] in module `%s': ", mem.memid.c_str(), idx, module->name.c_str());

		RTLIL::SigSpec clk = RTLIL::SigSpec(RTLIL::State::Sx);
		bool clk_polarity = 0;

		RTLIL::SigSpec sig_addr = port.addr;
		if (!find_sig_before_dff(sig_addr, clk, clk_polarity)) {
			log("no (compatible) $dff for address input found.\n");
			return;
		}

		RTLIL::SigSpec sig_data = port.data;
		if (!find_sig_before_dff(sig_data, clk, clk_polarity)) {
			log("no (compatible) $dff for data input found.\n");
			return;
		}

		RTLIL::SigSpec sig_en = port.en;
		if (!find_sig_before_dff(sig_en, clk, clk_polarity)) {
			log("no (compatible) $dff for enable input found.\n");
			return;
		}

		if (clk != RTLIL::SigSpec(RTLIL::State::Sx))
		{
			port.clk = clk;
			port.addr = sig_addr;
			port.data = sig_data;
			port.en = sig_en;
			port.clk_enable = true;
			port.clk_polarity = clk_polarity;
			mem.emit();

			log("merged $dff to port.\n");
			return;
		}

		log("no (compatible) $dff found.\n");
	}

	void handle_rd_port(Mem &mem, int idx)
	{
		auto &port = mem.rd_ports[idx];
		log("Checking read port `%s'[%d] in module `%s': ", mem.memid.c_str(), idx, module->name.c_str());

		FfData ff(&initvals);
		pool<std::pair<Cell *, int>> disconnect;
		if (find_sig_after_ff(port.data, ff, disconnect) && ff.has_clk)
		{
			if (ff.has_en && !ff.pol_en)
				ff.sig_en = module->LogicNot(NEW_ID, ff.sig_en);
			if (ff.has_arst && !ff.pol_arst)
				ff.sig_arst = module->LogicNot(NEW_ID, ff.sig_arst);
			if (ff.has_srst && !ff.pol_srst)
				ff.sig_srst = module->LogicNot(NEW_ID, ff.sig_srst);
			for (auto &it : disconnect) {
				Cell *cell = it.first;
				int idx = it.second;
				SigSpec q = cell->getPort(ID::Q);
				initvals.remove_init(q[idx]);
				q[idx] = module->addWire(stringf("$memory_dff_disconnected$%d", autoidx++));
				cell->setPort(ID::Q, q);
				fwd_merged_bits.insert(it);
			}
			port.clk = ff.sig_clk;
			port.clk_enable = true;
			port.clk_polarity = ff.pol_clk;
			if (ff.has_en)
				port.en = ff.sig_en;
			else
				port.en = State::S1;
			if (ff.has_arst) {
				port.arst = ff.sig_arst;
				port.arst_value = ff.val_arst;
			} else {
				port.arst = State::S0;
			}
			if (ff.has_srst) {
				port.srst = ff.sig_srst;
				port.srst_value = ff.val_srst;
				port.ce_over_srst = ff.ce_over_srst;
			} else {
				port.srst = State::S0;
			}
			port.init_value = ff.val_init;
			port.data = ff.sig_q;
			mem.emit();
			log("merged data $dff to cell.\n");
			return;
		}
	}

	void handle_rd_port_addr(Mem &mem, int idx)
	{
		auto &port = mem.rd_ports[idx];
		log("Checking read port address `%s'[%d] in module `%s': ", mem.memid.c_str(), idx, module->name.c_str());

		RTLIL::SigSpec clk_addr = RTLIL::SigSpec(RTLIL::State::Sx);
		bool clk_polarity = 0;
		RTLIL::SigSpec sig_addr = port.addr;
		if (find_sig_before_dff(sig_addr, clk_addr, clk_polarity) &&
				clk_addr != RTLIL::SigSpec(RTLIL::State::Sx))
		{
			for (int i = 0; i < GetSize(mem.wr_ports); i++) {
				auto &wport = mem.wr_ports[i];
				if (!wport.clk_enable || wport.clk != clk_addr || wport.clk_polarity != clk_polarity) {
					log("an address $dff was found, but is not compatible with write clock.\n");
					return;
				}
				port.transparency_mask[i] = true;
			}
			port.clk = clk_addr;
			port.en = State::S1;
			port.addr = sig_addr;
			port.clk_enable = true;
			port.clk_polarity = clk_polarity;
			mem.emit();
			log("merged address $dff to cell.\n");
			return;
		}

		log("no (compatible) $dff found.\n");
	}

	void run(bool flag_wr_only)
	{
		for (auto wire : module->wires()) {
			if (wire->port_output)
				for (auto bit : sigmap(wire))
					sigbit_users_count[bit]++;
		}

		for (auto cell : module->cells()) {
			if (cell->type.in(ID($dff), ID($dffe), ID($sdff), ID($sdffe), ID($sdffce), ID($adff), ID($adffe))) {
				SigSpec d = cell->getPort(ID::D);
				SigSpec q = cell->getPort(ID::Q);
				for (int i = 0; i < GetSize(d); i++)
					dff_sink[d[i]].insert(std::make_pair(cell, i));
				for (int i = 0; i < GetSize(q); i++)
					dff_driver[q[i]] = std::make_pair(cell, i);
			}
			for (auto &conn : cell->connections())
				if (!cell->known() || cell->input(conn.first))
					for (auto bit : sigmap(conn.second))
						sigbit_users_count[bit]++;
		}

		std::vector<Mem> memories = Mem::get_selected_memories(module);
		for (auto &mem : memories) {
			for (int i = 0; i < GetSize(mem.wr_ports); i++) {
				if (!mem.wr_ports[i].clk_enable)
					handle_wr_port(mem, i);
			}
		}

		if (!flag_wr_only) {
			for (auto &mem : memories) {
				for (int i = 0; i < GetSize(mem.rd_ports); i++) {
					if (!mem.rd_ports[i].clk_enable)
						handle_rd_port(mem, i);
				}
			}
			for (auto &mem : memories) {
				for (int i = 0; i < GetSize(mem.rd_ports); i++) {
					if (!mem.rd_ports[i].clk_enable)
						handle_rd_port_addr(mem, i);
				}
			}
		}
	}
};

struct MemoryDffPass : public Pass {
	MemoryDffPass() : Pass("memory_dff", "merge input/output DFFs into memories") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    memory_dff [options] [selection]\n");
		log("\n");
		log("This pass detects DFFs at memory ports and merges them into the memory port.\n");
		log("I.e. it consumes an asynchronous memory port and the flip-flops at its\n");
		log("interface and yields a synchronous memory port.\n");
		log("\n");
		log("    -nordfff\n");
		log("        do not merge registers on read ports\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		bool flag_wr_only = false;

		log_header(design, "Executing MEMORY_DFF pass (merging $dff cells to $memrd and $memwr).\n");

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-nordff" || args[argidx] == "-wr_only") {
				flag_wr_only = true;
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		for (auto mod : design->selected_modules()) {
			MemoryDffWorker worker(mod);
			worker.run(flag_wr_only);
		}
	}
} MemoryDffPass;

PRIVATE_NAMESPACE_END
