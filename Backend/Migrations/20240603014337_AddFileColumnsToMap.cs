using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Backend.Migrations
{
    /// <inheritdoc />
    public partial class AddFileColumnsToMap : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "Creator",
                table: "Maps");

            migrationBuilder.DropColumn(
                name: "SiteName",
                table: "Maps");

            migrationBuilder.AlterColumn<string>(
                name: "Name",
                table: "Maps",
                type: "TEXT",
                nullable: false,
                defaultValue: "",
                oldClrType: typeof(string),
                oldType: "TEXT",
                oldNullable: true);

            migrationBuilder.AddColumn<byte[]>(
                name: "PgmFile",
                table: "Maps",
                type: "BLOB",
                nullable: false,
                defaultValue: new byte[0]);

            migrationBuilder.AddColumn<string>(
                name: "Site",
                table: "Maps",
                type: "TEXT",
                nullable: false,
                defaultValue: "");

            migrationBuilder.AddColumn<DateTime>(
                name: "Timestamp",
                table: "Maps",
                type: "TEXT",
                nullable: false,
                defaultValue: new DateTime(1, 1, 1, 0, 0, 0, 0, DateTimeKind.Unspecified));

            migrationBuilder.AddColumn<byte[]>(
                name: "YamlFile",
                table: "Maps",
                type: "BLOB",
                nullable: false,
                defaultValue: new byte[0]);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "PgmFile",
                table: "Maps");

            migrationBuilder.DropColumn(
                name: "Site",
                table: "Maps");

            migrationBuilder.DropColumn(
                name: "Timestamp",
                table: "Maps");

            migrationBuilder.DropColumn(
                name: "YamlFile",
                table: "Maps");

            migrationBuilder.AlterColumn<string>(
                name: "Name",
                table: "Maps",
                type: "TEXT",
                nullable: true,
                oldClrType: typeof(string),
                oldType: "TEXT");

            migrationBuilder.AddColumn<string>(
                name: "Creator",
                table: "Maps",
                type: "TEXT",
                nullable: true);

            migrationBuilder.AddColumn<string>(
                name: "SiteName",
                table: "Maps",
                type: "TEXT",
                nullable: true);
        }
    }
}
